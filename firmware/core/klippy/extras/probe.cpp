#include "probe.h"
#include "Define.h"
#include "klippy.h"
#include "Define_config_path.h"
#define LOG_TAG "probe"
#undef LOG_LEVEL
#define LOG_LEVEL LOG_DEBUG
// Added for small post-probe dwell and warm-up behavior
#include <thread>
#include <chrono>
#include "log.h"
#define PROBE_STATE_CALLBACK_SIZE 16
static probe_state_callback_t probe_state_callback[PROBE_STATE_CALLBACK_SIZE];
int probe_state_callback_call(probe_data_t state);
PrinterProbe::PrinterProbe(std::string section_name, ProbeEndstopWrapperBase *mcu_probe)
{
    m_mcu_probe = mcu_probe;
    m_name = section_name;
    m_speed = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "speed", DEFAULT_PROBE_SPEED, DBL_MIN, DBL_MAX, 0.);                   // 探针第一次探测速度
    m_final_speed = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "final_speed", DEFAULT_PROBE_FINAL_SPEED, DBL_MIN, DBL_MAX, 0.); // 探针最后一次探测速度
    m_lift_speed = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "lift_speed", m_speed, DBL_MIN, DBL_MAX, 0.);                     // 抬升速度
    m_x_offset = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "x_offset", 0.);                                                    // 探针x偏移
    m_y_offset = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "y_offset", 0.);                                                    // 探针y偏移
    m_z_offset = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "z_offset", DEFAULT_PROBE_Z_OFFSET);                                // 探针z偏移
    m_z_offset = (0 - m_z_offset);
    m_z_offset_adjust = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "z_offset_adjust", DEFAULT_PROBE_Z_OFFSET_ADJUST);                    // z_offset之后的偏移
    move_after_each_sample = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "move_after_each_sample", DEFAULT_PROBE_MOVE_AFTER_EACH_SAMPLE); // 触发后继续下移的距离,没有这个参数就为零。
    m_probe_calibrate_z = 0.;
    m_multi_probe_pending = false;
    m_last_state = false;
    m_last_z_result = 0.;
    // Infer Z position to move to during a probe
    if (Printer::GetInstance()->m_pconfig->GetString(section_name, "stepper_z", "") != "") // 在probe中，如果有stepper_z，就用stepper_z的position_min，否则就用printer的minimum_z_position
    {
        m_z_position = Printer::GetInstance()->m_pconfig->GetDouble("stepper_z", "position_min", 0.);
    }
    else
    {
        m_z_position = Printer::GetInstance()->m_pconfig->GetDouble("printer", "minimum_z_position", 0.);
    }
    // Multi-sample support (for improved accuracy)
    m_sample_count = Printer::GetInstance()->m_pconfig->GetInt(section_name, "samples", DEFAULT_PROBE_SAMPLES, 1);                                                      // 样本数量
    m_edge_sample_count = Printer::GetInstance()->m_pconfig->GetInt(section_name, "edge_samples", DEFAULT_PROBE_SAMPLES, 1);                                       // 热床边缘增加的采点次数
    printf("%s m_edge_sample_count=%d\n", section_name.c_str(), m_edge_sample_count);
    m_edge_compensation = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "edge_compensation", 0.0);                                                      // 热床边缘补偿值
    printf("%s m_edge_compensatio=%f\n", section_name.c_str(), m_edge_compensation);
    m_sample_retract_dist = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "sample_retract_dist", DEFAULT_PROBE_SAMPLE_RETRACT_DIST, DBL_MIN, DBL_MAX, 0.); // 每个样本之间的抬升间距
    m_samples_result = Printer::GetInstance()->m_pconfig->GetString(section_name, "samples_result", DEFAULT_PROBE_SAMPLES_RESULT);                                      // 多个样本的处理方式
    m_samples_tolerance = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "samples_tolerance", DEFAULT_PROBE_SAMPLES_TOLERANCE, 0.);                         // 样本间的最大差值
    m_samples_tolerance_step = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "samples_tolerance_step", 0.005, 0.);                  // 样本间的最大差值步进
    m_edge_samples_tolerance = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "edge_samples_tolerance", 0.03, 0.);                     // 热床边缘样本间的最大差值
    m_edge_samples_tolerance_step = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "edge_samples_tolerance_step", 0.005, 0.);                  // 热床边缘样本间的最大差值步进

    m_fast_autoleveling_threshold = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "fast_autoleveling_threshold", DEFAULT_PROBE_FAST_AUTOLEVELING_THRESHOLD, 0.);                         // 快速调平阈值：与上一次该点采样值的误差
    m_samples_retries = Printer::GetInstance()->m_pconfig->GetInt(section_name, "samples_tolerance_retries", DEFAULT_PROBE_SAMPLES_RETRIES, 0);                         // 最大重试次数

    // Register homing event handlers
    Printer::GetInstance()->register_event_homing_move_handler("homing:homing_move_begin:PrinterProbe:" + section_name, std::bind(&PrinterProbe::_handle_homing_move_begin, this, std::placeholders::_1));
    Printer::GetInstance()->register_event_homing_move_handler("homing:homing_move_end:PrinterProbe:" + section_name, std::bind(&PrinterProbe::_handle_homing_move_end, this, std::placeholders::_1));
    Printer::GetInstance()->register_event_homing_handler("homing:home_rails_begin:PrinterProbe:" + section_name, std::bind(&PrinterProbe::_handle_home_rails_begin, this, std::placeholders::_1, std::placeholders::_2));
    Printer::GetInstance()->register_event_homing_handler("homing:home_rails_end:PrinterProbe:" + section_name, std::bind(&PrinterProbe::_handle_home_rails_end, this, std::placeholders::_1, std::placeholders::_2));
    Printer::GetInstance()->register_event_handler("gcode:command_error:PrinterProbe" + section_name, std::bind(&PrinterProbe::_handle_command_error, this));
    if (section_name == "bed_mesh_probe") // 如果是床网探针实例，就不注册下面的命令。源码只有一个探针实例，魔改。
    {
        m_z_offset = Printer::GetInstance()->m_pconfig->GetDouble("probe", "z_offset", DEFAULT_PROBE_Z_OFFSET); // 与probe的z_offset共用一个参数
        m_z_offset = (0 - m_z_offset);
        m_z_offset_adjust = Printer::GetInstance()->m_pconfig->GetDouble("probe", "z_offset_adjust", DEFAULT_PROBE_Z_OFFSET_ADJUST); // z_offset之后的偏移
        Printer::GetInstance()->m_gcode->register_command("HOME_ACCURACY", std::bind(&PrinterProbe::cmd_PROBE_ACCURACY, this, std::placeholders::_1), false, "");
        return;
    }
    // Register PROBE/QUERY_PROBE commands
    cmd_PROBE_help = "PrinterProbe Z-height at current XY position";
    cmd_QUERY_PROBE_help = "Return the status of the z-probe";
    cmd_PROBE_CALIBRATE_help = "Calibrate the probe's z_offset";
    cmd_PROBE_ACCURACY_help = "PrinterProbe Z-height accuracy at current XY position";
    Printer::GetInstance()->m_gcode->register_command("PROBE", std::bind(&PrinterProbe::cmd_PROBE, this, std::placeholders::_1), false, cmd_PROBE_help);
    // Printer::GetInstance()->m_gcode->register_command("G30", std::bind(&PrinterProbe::cmd_PROBE, this, std::placeholders::_1), false, cmd_PROBE_help);
    Printer::GetInstance()->m_gcode->register_command("QUERY_PROBE", std::bind(&PrinterProbe::cmd_QUERY_PROBE, this, std::placeholders::_1), false, cmd_QUERY_PROBE_help);
    Printer::GetInstance()->m_gcode->register_command("PROBE_CALIBRATE", std::bind(&PrinterProbe::cmd_PROBE_CALIBRATE, this, std::placeholders::_1), false, cmd_PROBE_CALIBRATE_help);
    Printer::GetInstance()->m_gcode->register_command("PROBE_ACCURACY", std::bind(&PrinterProbe::cmd_PROBE_ACCURACY, this, std::placeholders::_1), false, cmd_PROBE_ACCURACY_help);
}

PrinterProbe::~PrinterProbe()
{
}

void PrinterProbe::_handle_homing_move_begin(HomingMove *hmove)
{
    if (m_mcu_probe->m_mcu_endstop == hmove->get_mcu_endstops()[0])
    {
        m_mcu_probe->probe_prepare(hmove);
    }
}

void PrinterProbe::_handle_homing_move_end(HomingMove *hmove)
{
    if (m_mcu_probe->m_mcu_endstop == hmove->get_mcu_endstops()[0])
    {
        m_mcu_probe->probe_finish(hmove);
    }
}

void PrinterProbe::_handle_home_rails_begin(Homing *homing_state, std::vector<PrinterRail *> rails)
{
    for (int i = 0; i < rails.size(); i++)
    {
        std::vector<MCU_endstop *> endstops = rails[i]->m_endstops;
        if (m_mcu_probe->m_mcu_endstop == endstops[0])
        {
            multi_probe_begin();
        }
    }
}

void PrinterProbe::_handle_home_rails_end(Homing *homing_state, std::vector<PrinterRail *> rails)
{
    for (int i = 0; i < rails.size(); i++)
    {
        std::vector<MCU_endstop *> endstops = rails[i]->m_endstops;
        if (m_mcu_probe->m_mcu_endstop == endstops[0])
        {
            multi_probe_end();
        }
    }
}

void PrinterProbe::_handle_command_error()
{
    // try: //---??---
    //     self.multi_probe_end()
    // except:
    //     logging.exception("Multi-probe end")
}
void PrinterProbe::multi_probe_begin()
{
    m_mcu_probe->multi_probe_begin();
    m_multi_probe_pending = true;
}

void PrinterProbe::multi_probe_end()
{
    if (m_multi_probe_pending)
    {
        m_multi_probe_pending = false;
        m_mcu_probe->multi_probe_end();
    }
}

ProbeEndstopWrapperBase *PrinterProbe::setup_pin(std::string pin_type, pinParams pin_params)
{
    return m_mcu_probe;
}

double PrinterProbe::get_lift_speed(GCodeCommand *gcmd)
{
    if (gcmd != NULL)
    {
        gcmd->get_double("LIFT_SPEED", m_lift_speed, DBL_MIN, DBL_MAX, 0.0);
    }
    return m_lift_speed;
}

std::vector<double> PrinterProbe::get_offsets()
{
    std::vector<double> ret = {m_x_offset, m_y_offset, m_z_offset};
    return ret;
}

std::vector<double> PrinterProbe::_probe(double speed)
{
    double curtime = get_monotonic();
    // if 'z' not in toolhead.get_status(curtime)['homed_axes']:
    //         raise self.printer.command_error("Must home before probe")  //---??---
    std::vector<double> pos = Printer::GetInstance()->m_tool_head->get_position();
    pos[2] = m_z_position; // 向下探测最长距离
    std::vector<double> epos = Printer::GetInstance()->m_printer_homing->probing_move(m_mcu_probe->m_mcu_endstop, pos, speed);
    LOG_D("probe at %.3f,%.3f is z=%.6f\n", epos[0], epos[1], epos[2]);
    return epos;
}

void PrinterProbe::_move(std::vector<double> coord, double speed)
{
    Printer::GetInstance()->m_tool_head->manual_move(coord, speed);
}

std::vector<double> PrinterProbe::_calc_mean(std::vector<std::vector<double>> positions)
{
    int count = positions.size();
    double sum_x = 0., sum_y = 0., sum_z = 0., sum_compensation_z = 0.;
    for (int i = 0; i < positions.size(); i++)
    {
        std::vector<double> position = positions[i];
        sum_x += position[0];
        sum_y += position[1];
        sum_z += position[2];
        // sum_compensation_z += position[3];
        // LOG_D("z:%d %f\n",i,position[2]);
        printf("z:%d %f %f\n", i, position[2], position[3]);
    }
    double average_x = sum_x / count;
    double average_y = sum_y / count;
    double average_z = sum_z / count;
    // double average_compensation_z = sum_compensation_z / count;
    std::vector<double> ret = {average_x, average_y, average_z};
    // printf("mean z:%f(%f)\n",average_z,average_compensation_z);
    printf("mean z:%f(%f)\n", average_z);
    return ret;
}

std::vector<double> PrinterProbe::_calc_median(std::vector<std::vector<double>> positions)
{
    sort(positions.begin(), positions.end(), [](const std::vector<double> &a, const std::vector<double> &b)
         { return a[2] < b[2]; });
    int middle = positions.size() / 2;
    if ((positions.size() & 1) == 1)
    {
        return positions[middle];
    }
    std::vector<std::vector<double>> pos_middle;
    pos_middle.push_back(positions[middle - 1]);
    pos_middle.push_back(positions[middle]);
    return _calc_mean(pos_middle);
}
std::vector<double> PrinterProbe::_calc_weighted_average(std::vector<std::vector<double>> positions) // 一定是探测两次才使用
{
    int count = positions.size();
    if (count != 2)
    {
        return _calc_mean(positions);
    }
    double sum_x = 0.0f, sum_y = 0.0f;
    for (int i = 0; i < positions.size(); i++) // X，Y采取平均值，Z采取加权平均值
    {
        std::vector<double> position = positions[i];
        sum_x += position[0];
        sum_y += position[1];
    }
    double average_x = sum_x / count;
    double average_y = sum_y / count;
    double average_z = (positions[1][2] * 3.0f + positions[0][2] * 2.0f) * 0.2f;
    printf("weighted_average z0:%f z1:%f\n", positions[0][2], positions[1][2]);
    std::vector<double> ret = {average_x, average_y, average_z};
    printf("weighted_average z:%d %f\n", average_z);
    return ret;
}
std::vector<double> PrinterProbe::run_probe(GCodeCommand &gcmd)
{
    double speed = 0.0f;
    std::vector<double> coord = {0, 0, 0}; // 发送给mcu的坐标临时变量。
    double lift_speed = get_lift_speed(&gcmd);
    int sample_count = gcmd.get_int("SAMPLES", m_sample_count, 1);
    double sample_retract_dist = gcmd.get_double("SAMPLE_RETRACT_DIST", m_sample_retract_dist, DBL_MIN, DBL_MAX, 0.);
    double samples_tolerance = gcmd.get_double("SAMPLES_TOLERANCE", m_samples_tolerance, 0.);
    double samples_tolerance_step = m_samples_tolerance_step;
    int samples_retries = gcmd.get_int("SAMPLES_TOLERANCE_RETRIES", m_samples_retries, 0);
    std::string samples_result = gcmd.get_string("SAMPLES_RESULT", m_samples_result);
    bool must_notify_multi_probe = !m_multi_probe_pending;
    // static double last_compensation_z_measure = 100000;
        // Small dwell and optional warm-up for optical probes:
        // - probe_dwell_ms: sleep after probe to allow sensor to settle
        // - performed_warmup: per-run warm-up discard when using multiple samples
        const int probe_dwell_ms = 5; // ms; adjust for your specific optical hardware
        bool performed_warmup = false;
    if (must_notify_multi_probe)
    {
        multi_probe_begin();
    }
    std::vector<double> probexy = Printer::GetInstance()->m_tool_head->get_position();
    int retries = 0;
    std::vector<std::vector<double>> positions;
    std::vector<double> z_positions;
    std::vector<double> z_compensation_positions;
    m_error = 0;
    bool edge_flag = false;
    // 边缘区域采点次数： x < 30  || x > 200  y < 10  y > 230
    if (probexy[0] <= 20 || probexy[0] >= 230 || probexy[1] <= 20 || probexy[1] >= 230 ) {
        sample_count = m_edge_sample_count;
        edge_flag = true;
        samples_tolerance = m_edge_samples_tolerance;
        samples_tolerance_step = m_edge_samples_tolerance_step;
    }
    int probe_index = sample_count;
    while (positions.size() < sample_count)
    {
        probe_index--;
        if (positions.size() == sample_count - 1) // 最后一次探测
        {
            speed = m_final_speed;
        }
        else
        {
            speed = gcmd.get_double("PROBE_SPEED", m_speed, DBL_MIN, DBL_MAX, 0.0);
        }
		
		double requested_speed = speed;
		speed = 1.0;  //clamp Z descent to this to hopefully improve accuracy 
        LOG_W("Probe descent speed clamped: requested=%.3f mm/s -> using %.3f mm/s\n",
              requested_speed, speed);
		
        LOG_D("准备执行探测,当前Z 坐标%.6f,探测速度为%.2f\n", Printer::GetInstance()->m_tool_head->get_position()[2], speed); // 获取的这个位置是应用了position_endstop之后的坐标
        // 执行 run_G29_Z
        std::vector<double> pos = _probe(speed);                                                                              // 采集样本
        // --- Added: short dwell to allow optical probe to stabilize ---
        std::this_thread::sleep_for(std::chrono::milliseconds(probe_dwell_ms));

        // --- Added: flush last move time (ensure moves completed/lookahead flushed) ---
        Printer::GetInstance()->m_tool_head->get_last_move_time();

        // Perform a single warm-up probe for multi-sample runs:
        // helps optical probes stabilize thermally/optically at the start.
        if (!performed_warmup && sample_count > 1) {
            performed_warmup = true;
            (void)_probe(speed); // discard warm-up result
            std::this_thread::sleep_for(std::chrono::milliseconds(probe_dwell_ms));
        }
        double z_pos = pos[2];
        std::vector<double> position = {pos[0], pos[1], z_pos};

        positions.push_back(position);                                     // 保存样本
        z_positions.push_back(pos[2]);                                     // 保存样本的z值
        z_compensation_positions.push_back(pos[3]);                        // 保存样本的compensation z值
        double max = *max_element(z_positions.begin(), z_positions.end()); // 获取样本中的最大值
        // double compensation_max = *max_element(z_compensation_positions.begin(), z_compensation_positions.end()); // 获取样本中的最大值
        double min = *min_element(z_positions.begin(), z_positions.end()); // 获取样本中的最小值
        // double compensation_min = *min_element(z_compensation_positions.begin(), z_compensation_positions.end()); // 获取样本中的最小值
        double now_z_measure = pos[2]; // - m_z_offset;
        // double now_compensation_z_measure = pos[3] - m_z_offset;
        // double delta_compensation_last = std::fabs(now_compensation_z_measure-last_compensation_z_measure);
        if ((max - min) > samples_tolerance) // 采集的样本中，最大值和最小值的差值超过容差  与上一次值查过 0.5
        {
            if (retries >= samples_retries) // 失败，重试次数超过最大值
            {
                LOG_E("PrinterProbe samples exceed samples_tolerance MAX:%f MIN:%f samples_tolerance:%f\n", max, min, samples_tolerance);
                m_error = 1;
                throw std::runtime_error("Probe samples exceed samples_tolerance");
            }
            LOG_D("PrinterProbe samples exceed tolerance. Retrying... MAX:%f MIN:%f samples_tolerance:%f\n", max, min, samples_tolerance);

            retries += 1;
            if (retries >= 2)
                samples_tolerance += samples_tolerance_step;
            LOG_I("PrinterProbe samples_tolerance:%f\n", samples_tolerance);
            std::vector<std::vector<double>>().swap(positions); // 清空样本
            std::vector<double>().swap(z_positions);            // 清空样本的z值
        }
        if (positions.size() < sample_count) // 还没采集够数量的样本
        {
            if (!Printer::GetInstance()->m_strain_gauge->m_cfg->m_use_probe_by_step)
            {
                LOG_D("\n第%d次采集样本成功,触发Z,相对归零后逻辑零点位置为: %.6f", positions.size(), now_z_measure);
                // coord = {probexy[0], probexy[1], z_pos + move_after_each_sample*probe_index/sample_count};
                // LOG_D("触发后继续下移:%.6f mm\n", move_after_each_sample);
                // _move(coord, speed);
                coord = {probexy[0], probexy[1], z_pos + sample_retract_dist * (double)probe_index / (double)sample_count}; // 抬起，准备下一次探针触发
                LOG_D("抬起:%f 后的目标绝对位置Z为:%.6f，移动到下一探测起点\n", sample_retract_dist, z_pos + sample_retract_dist * (double)probe_index / (double)sample_count);
                // 移动到下一次探测点坐标
                _move(coord, lift_speed);
            }
        }
        else
        {
            LOG_D("最后一次采集样本成功,触发Z,相对归零后逻辑零点位置为%.6f 回退：%.3f \n", now_z_measure,sample_retract_dist);
            coord = {probexy[0], probexy[1], z_pos + sample_retract_dist};
            // LOG_D("触发后继续下移:%.6f mm\n", sample_retract_dist);
            // _move(coord, speed);
        }
    }
    if (must_notify_multi_probe)
    {
        multi_probe_end();
    }
    std::vector<double> last_ret={0,0,0};
    if (samples_result == "median")
    {
        LOG_D("run_probe median\n");
        last_ret = _calc_median(positions);
    }
    // ??
    else if (samples_result == "weighted") // 采样结果为z值的加权平均值
    {
        LOG_D("run_probe weighted\n");
        last_ret = _calc_weighted_average(positions);
    }else {
        LOG_D("run_probe mean.......................................................................................................................................................\n");
        last_ret = _calc_mean(positions);
    }
    if (edge_flag) {
        last_ret[2] = last_ret[2] + m_edge_compensation;
        LOG_D("-------------edge_flag %f--------------------------------------------\n",last_ret[2]);
    }
    return last_ret;
}

void PrinterProbe::cmd_PROBE(GCodeCommand &gcmd)
{
    try
    {
        m_last_state = false;
        std::vector<double> pos = run_probe(gcmd);
        m_last_z_result = pos[2];
        m_last_state = true;
        LOG_I("Probe Result is %.6f\n", m_last_z_result);
        probe_state_callback_call(probe_data_t{.state = CMD_BEDMESH_PROBE_SUCC});
    }
    catch (std::exception &e)
    {
        m_last_state = false;
        m_last_z_result = 0.0;
        LOG_E("Probe error:%s\n", e.what());
        if (strstr(e.what(), "No response from MCU") != nullptr)
            throw;
        probe_state_callback_call(probe_data_t{.state = CMD_BEDMESH_PROBE_EXCEPTION});
        throw;
    }
    // gcmd.respond_info("Result is z=%.6f" % (pos[2],)) //---??---
}

void PrinterProbe::cmd_QUERY_PROBE(GCodeCommand &gcmd)
{
    double print_time = Printer::GetInstance()->m_tool_head->get_last_move_time();
    // bool res = m_mcu_probe->m_mcu_endstop->query_endstop(print_time);
    // m_last_state = res;
    // gcmd.respond_info("probe: %s" % (["open", "TRIGGERED"][not not res],)) //---??---
}

struct probe_state PrinterProbe::get_status(double eventtime)
{
    struct probe_state ret = {m_last_state, m_last_z_result};
    return ret;
}
void PrinterProbe::cmd_PROBE_ACCURACY(GCodeCommand &gcmd)
{
    double speed = gcmd.get_float("PROBE_SPEED", m_speed, DBL_MIN, DBL_MAX, 0.0);
    double lift_speed = get_lift_speed(&gcmd);
    int sample_count = gcmd.get_int("SAMPLES", 10, 1);
    double sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST", m_sample_retract_dist, DBL_MIN, DBL_MAX, 0.);
    std::vector<double> pos = Printer::GetInstance()->m_tool_head->get_position();
    pos[2] = 0;
    _move(pos, lift_speed);
    printf("PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f (samples=%d retract=%.3f speed=%.1f lift_speed=%.1f)\n", pos[0], pos[1], pos[2], sample_count, sample_retract_dist, speed, lift_speed);
    multi_probe_begin();
    std::vector<std::vector<double>> positions;
    double last_z_position = 0;
    // int i=0;
    while (positions.size() < sample_count)
    {
        std::vector<double> pos = _probe(speed);
        // std::vector<double> position = {pos[0], pos[1], pos[2]};
        // double delta = std::fabs(pos[2]-last_z_position);
        // if (i>0 && delta>0.05 ){
        //     printf("delta > 0.05 delta:%f now:%f last:%f\n",delta,pos[2],last_z_position);
        // }else {
        //     position[2] = (last_z_position + pos[2])/2;
            positions.push_back(pos);
        // }
        // last_z_position = pos[2];
        // std::vector<double> liftpos = {DBL_MIN, DBL_MIN, pos[2] + sample_retract_dist};
        // _move(liftpos, lift_speed);
        // i++;
    }
    multi_probe_end();
    // Calculate maximum, minimum and average values
    std::vector<double> z_position;
    for (int i = 0; i < positions.size(); i++)
    {
        z_position.push_back(positions[i][2]);
    }
    double max_value = *max_element(z_position.begin(), z_position.end());
    double min_value = *min_element(z_position.begin(), z_position.end());
    double range_value = max_value - min_value;
    double avg_value = _calc_mean(positions)[2];
    double median = _calc_median(positions)[2];
    // calculate the standard deviation
    int deviation_sum = 0;
    for (int i = 0; i < positions.size(); i++)
    {
        deviation_sum += pow(positions[i][2] - avg_value, 2);
    }
    double sigma = pow(deviation_sum / positions.size(), 0.5);
    printf("\n\n\n\nprobe accuracy results: maximum %.6f, minimum %.6f, range %.6f, average %.6f, median %.6f, standard deviation %.6f\n\n\n\n\n",
           max_value, min_value, range_value, avg_value, median, sigma);
}
void PrinterProbe::probe_calibrate_finalize()
{
    double z_offset = Printer::GetInstance()->m_probe->m_z_offset - m_probe_calibrate_z - m_z_offset_adjust;
    // 与klipper源码不同，这里没有手动校准步骤
    Printer::GetInstance()->m_pconfig->SetDouble("probe", "z_offset", (0.0f - z_offset));
    Printer::GetInstance()->m_pconfig->SetDouble("stepper_z", "position_endstop", z_offset);
    Printer::GetInstance()->m_probe->m_z_offset = z_offset;
    Printer::GetInstance()->m_bed_mesh_probe->m_z_offset = z_offset;
    Printer::GetInstance()->m_tool_head->m_kin->m_rails[2]->m_position_endstop = z_offset;
    // std::cout << m_name << "  PROBE_CALIBRATE z_offset=  " << z_offset << std::endl;
    printf("probe_calibrate_finalize: z_offset: %.3f\n"
           "The SAVE_CONFIG command will update the printer config file\n"
           "with the above and restart the printer.",
           z_offset);
    Printer::GetInstance()->m_pconfig->WriteIni(CONFIG_PATH); // 写入配置
}

void PrinterProbe::cmd_PROBE_CALIBRATE(GCodeCommand &gcmd)
{
    // verify_no_manual_probe();
    // Perform initial probe
    double lift_speed = get_lift_speed(&gcmd);
    try
    {
        std::vector<double> curpos = run_probe(gcmd);
        printf("PrinterProbe cmd_PROBE_CALIBRATE Result is z=%.6f\n", curpos[2]);
        /*获取到新的Z_offset后修改当前Z坐标，不用在归零后才应用*/
        std::vector<double> cur_commanded_pos = Printer::GetInstance()->m_tool_head->m_commanded_pos;
        LOG_D("修改前 cur_commanded_pos[2] = %.6f\n", cur_commanded_pos[2]);
        // Do not change commanded Z here; Klipper-style behavior:
        // calibration only measures and later updates config via probe_calibrate_finalize().
        LOG_D("Calibration sample = %.6f (no commanded Z modification)\n", curpos[2]);
        /*获取到新的Z_offset后修改当前Z坐标，不用在归零后才应用*/
        // Move away from the bed
        m_probe_calibrate_z = curpos[2];
        curpos[2] += m_sample_retract_dist;
        std::vector<double> curpos_temp = {curpos[0], curpos[1], curpos[2]};
        _move(curpos_temp, lift_speed);
        // Move the nozzle over the probe point
        curpos_temp[0] += m_x_offset;
        curpos_temp[1] += m_y_offset;
        _move(curpos_temp, lift_speed);

        // Start manual probe
        // 与klipper源码不同，这里没有手动校准步骤
        probe_calibrate_finalize();
        //   ManualProbeHelper(self.printer, gcmd, m_probe_calibrate_finalize); //---??---
        probe_state_callback_call(probe_data_t{.state = CMD_BEDMESH_PROBE_SUCC});
    }
    catch (std::exception &e)
    {
        LOG_E("Probe error:%s\n", e.what());
        if (strstr(e.what(), "No response from MCU") != nullptr)
            throw;
        probe_state_callback_call(probe_data_t{.state = CMD_BEDMESH_PROBE_EXCEPTION});
        throw;
    }
}

ProbeEndstopWrapper::ProbeEndstopWrapper(std::string section_name)
{
    m_position_endstop = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "z_offset", DBL_MIN);
    m_stow_on_each_sample = Printer::GetInstance()->m_pconfig->GetBool(section_name, "deactivate_on_each_sample", true);
    // gcode_macro = self.printer.load_object(config, 'gcode_macro')  // ---??---  模块需要添加
    // std::string activate_gcode = gcode_macro.load_template(
    //     config, 'activate_gcode', '')
    // self.deactivate_gcode = gcode_macro.load_template(
    //     config, 'deactivate_gcode', '')
    // Create an "endstop" object to handle the probe pin
    std::string pin = Printer::GetInstance()->m_pconfig->GetString(section_name, "pin", "");
    pinParams *pin_params = Printer::GetInstance()->m_ppins->lookup_pin(pin, true, true);
    m_mcu_endstop = new MCU_endstop(Printer::GetInstance()->m_mcu, pin_params); // 创建一个endstop对象来处理探针引脚，发命令给mcu初始化探针引脚。
    if (section_name == "bed_mesh_probe")
    {
        Printer::GetInstance()->register_event_handler("klippy:mcu_identify:bed_mesh_probe", std::bind(&ProbeEndstopWrapper::handle_mcu_identify, this));
    }
    else
    {
        Printer::GetInstance()->register_event_handler("klippy:mcu_identify", std::bind(&ProbeEndstopWrapper::handle_mcu_identify, this));
    }
    // multi probes state
    m_multi = "OFF";
}

ProbeEndstopWrapper::~ProbeEndstopWrapper()
{
}

void ProbeEndstopWrapper::handle_mcu_identify()
{
    std::vector<std::vector<MCU_stepper *>> steppers = Printer::GetInstance()->m_tool_head->m_kin->get_steppers();
    for (int i = 0; i < steppers.size(); i++)
    {
        std::vector<MCU_stepper *> steppers_temp = steppers[i];
        for (int j = 0; j < steppers_temp.size(); j++)
        {
            if (steppers_temp[j]->is_active_axis('z'))
            {
                m_mcu_endstop->add_stepper(steppers_temp[j]);
            }
        }
    }
}
void ProbeEndstopWrapper::raise_probe()
{
    std::vector<double> start_pos = Printer::GetInstance()->m_tool_head->get_position();
    // self.deactivate_gcode.run_gcode_from_command() // ---??---
    std::vector<double> pos = Printer::GetInstance()->m_tool_head->get_position();
    if (pos[0] != start_pos[0] || pos[1] != start_pos[1] || pos[2] != start_pos[2])
    {
        std::cout << "Toolhead moved during probe activate_gcode script" << std::endl;
    }
}

void ProbeEndstopWrapper::lower_probe()
{
    std::vector<double> start_pos = Printer::GetInstance()->m_tool_head->get_position();
    // self.activate_gcode.run_gcode_from_command() // ---??---
    std::vector<double> pos = Printer::GetInstance()->m_tool_head->get_position();
    if (pos[0] != start_pos[0] || pos[1] != start_pos[1] || pos[2] != start_pos[2])
    {
        std::cout << "Toolhead moved during probe deactivate_gcode script" << std::endl;
    }
}

void ProbeEndstopWrapper::multi_probe_begin()
{
    if (m_stow_on_each_sample)
    {
        return;
    }
    m_multi = "FIRST";
}

void ProbeEndstopWrapper::multi_probe_end()
{
    if (m_stow_on_each_sample)
    {
        return;
    }
    raise_probe();
    m_multi = "OFF";
}

void ProbeEndstopWrapper::probe_prepare(HomingMove *hmove)
{
    if (m_multi == "OFF" || m_multi == "FIRST")
    {
        lower_probe();
        if (m_multi == "FIRST")
        {
            m_multi = "OFF";
        }
    }
}

void ProbeEndstopWrapper::probe_finish(HomingMove *hmove)
{
    if (m_multi == "OFF")
    {
        raise_probe();
    }
}

double ProbeEndstopWrapper::get_position_endstop()
{
    return m_position_endstop;
}

ProbePointsHelper::ProbePointsHelper(std::string section_name, std::function<std::string(std::vector<double>, std::vector<std::vector<double>>)> finalize_callback, std::vector<std::vector<double>> default_points)
{
    m_finalize_callback = finalize_callback;
    m_probe_points.assign(default_points.begin(), default_points.end());
    m_name = section_name;
    // Read config settings

    if (default_points.size() == 0 || Printer::GetInstance()->m_pconfig->GetString(section_name, "points", "") != "")
    {
        std::string points = Printer::GetInstance()->m_pconfig->GetString(section_name, "points", "");
        std::istringstream iss(points);  // 输入流
        std::string line;                // 接收缓冲区
        while (getline(iss, line, '\n')) // 以split为分隔符
        {
            std::string point;
            std::istringstream iss(line); // 输入流
            std::vector<double> out;
            while (getline(iss, point, ','))
            {
                out.push_back(atof(point.c_str()));
            }
            m_probe_points.push_back(out);
        }
    }

    // m_fast_probe_points.clear();
    // m_fast_probe_points.push_back(m_probe_points.at(0));
    // m_fast_probe_points.push_back(m_probe_points.at(5));
    // m_fast_probe_points.push_back(m_probe_points.at(m_probe_points.size()-6));
    // m_fast_probe_points.push_back(m_probe_points.at(m_probe_points.size()-1));

    m_horizontal_move_z = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "horizontal_move_z", 5.);
    m_speed = Printer::GetInstance()->m_pconfig->GetDouble(section_name, "speed", 50., DBL_MIN, DBL_MAX, 0.);
    m_use_offsets = false;
    // Internal probing state
    m_lift_speed = m_speed;
    m_probe_offsets[0] = 0;
    m_probe_offsets[1] = 0;
    m_probe_offsets[2] = 0;
}

ProbePointsHelper::~ProbePointsHelper()
{
}

void ProbePointsHelper::minimum_points(int n)
{
    if (m_probe_points.size() < n)
    {
        std::cout << "Need at least %d probe points for " << m_name << std::endl;
    }
}

void ProbePointsHelper::update_probe_points(std::vector<std::vector<double>> points, int min_points)
{
    m_probe_points.assign(points.begin(), points.end());
    minimum_points(min_points);
}

void ProbePointsHelper::use_xy_offsets(bool use_offsets)
{
    m_use_offsets = use_offsets;
}
double ProbePointsHelper::get_lift_speed()
{
    return m_lift_speed;
}
bool ProbePointsHelper::_move_next(int move_type)
{
    std::cout << "_move_next :" << move_type << "........................................................................." << std::endl;
    // Lift toolhead
    double speed = m_lift_speed;
    // if (m_results.size() == 0)
    // {
    //     // Use full speed to first probe position
    //     speed = m_speed;
    // }
    std::vector<double> pos = {DBL_MIN, DBL_MIN, m_horizontal_move_z};
    Printer::GetInstance()->m_tool_head->manual_move(pos, speed);
    std::cout << __func__ << " z move pos " << pos[2] << std::endl;
    // Check if done probing
    int probe_points_size = 0;
    if (move_type==2){
        probe_points_size = m_fast_probe_points.size();
    } else {
        probe_points_size = m_probe_points.size();
    }

    if (m_results.size() >= probe_points_size)
    {
        Printer::GetInstance()->m_tool_head->get_last_move_time();
        std::string res = m_finalize_callback(m_probe_offsets, m_results); // bed_mesh 完成后的回调处理函数
        std::cout << " ..............................................................................m_finalize_callback res:" << res << std::endl;
        if (res != "retry")
        {
            return true;
        }
        std::vector<std::vector<double>>().swap(m_results);
    }
    // Move to next XY probe point
    std::vector<double> nextpos;
    if (move_type==2){
        nextpos = m_fast_probe_points[m_results.size()]; // m_results是触发点的坐标。
    } else {
        nextpos = m_probe_points[m_results.size()]; 
    }
    if (m_use_offsets)
    {
        nextpos[0] -= m_probe_offsets[0];
        nextpos[1] -= m_probe_offsets[1];
    }
    nextpos.push_back(m_horizontal_move_z);
    Printer::GetInstance()->m_tool_head->manual_move(nextpos, m_speed);
    return false;
}

void ProbePointsHelper::start_probe(GCodeCommand &gcmd)
{
    verify_no_manual_probe();
    PrinterProbe *probe = nullptr;
    if (gcmd.m_command.find("BED_MESH_CALIBRATE") != std::string::npos) // 换专属探针
    {
        probe = Printer::GetInstance()->m_bed_mesh_probe;
    }
    else
    {
        probe = Printer::GetInstance()->m_probe;
    }
    std::cout << "start_probe" << std::endl;
    std::string method = gcmd.get_string("METHOD", "automatic");
    std::vector<std::vector<double>>().swap(m_results);
    if (method == "fast" && m_last_fast_probe_points_z.size() > 0 && m_fast_probe_points.size() > 0 )
    {
        std::cout << "cmd_BED_MESH_CALIBRATE fast" << std::endl;
        // fast probe
        m_lift_speed = m_speed;
        m_probe_offsets[0] = 0;
        m_probe_offsets[1] = 0;
        m_probe_offsets[2] = 0;
        // 快速调平
        if (_fast_probe_start(probe,gcmd)){
            // 快速调平 ok ，直接返回
            LOG_W("fast auto leveling succssful，Use the previous leveling result\n");
            return;
        }
        LOG_W("fast auto leveling failed, continue auto leveling step\n");
        Printer::GetInstance()->m_gcode_io->single_command("BED_MESH_SET_INDEX INDEX=1");
    }
    if (probe == NULL || method == "manual")
    {
        std::cout << "cmd_BED_MESH_CALIBRATE manual" << std::endl;
        // Manual probe
        m_lift_speed = m_speed;
        m_probe_offsets[0] = 0;
        m_probe_offsets[1] = 0;
        m_probe_offsets[2] = 0;
        _manual_probe_start();
        return;
    }
    std::cout << "cmd_BED_MESH_CALIBRATE automatic" << std::endl;
    // Perform automatic probing
    m_lift_speed = probe->get_lift_speed(&gcmd);
    std::vector<double> ret = probe->get_offsets(); // 获取 probe 段的 x,y,z 偏移量
    m_probe_offsets[0] = ret[0];
    m_probe_offsets[1] = ret[1];
    m_probe_offsets[2] = ret[2];
    if (m_horizontal_move_z < m_probe_offsets[2])
    {
        std::cout << "horizontal_move_z can't be less than probe's z_offset" << std::endl;
    }
    probe->multi_probe_begin();
    while (1)
    {
        // 移动到下一点：自动调平
        bool done = _move_next(0);
        if (done)
        {
            if (probe == Printer::GetInstance()->m_bed_mesh_probe)
            {
                probe_state_callback_call(probe_data_t{.state = CMD_BEDMESH_PROBE_SUCC});
            }
            break;
        }
        try
        {
            std::vector<double> pos = probe->run_probe(gcmd);
            m_results.push_back(pos);
            probe_data_t probe_event;
            probe_event.state = AUTO_LEVELING_PROBEING;
            probe_event.value = pos[2];
            probe_state_callback_call(probe_event);
            LOG_D("当前触发点的坐标为: %f, %f, %f(%f)\n", pos[0], pos[1], pos[2], pos[3]);
        }
        catch (std::exception &e)
        {
            Printer::GetInstance()->invoke_shutdown("");
            if (strstr(e.what(), "No response from MCU") != nullptr)
                throw;
            LOG_E("bed mesh probe error\n");
            if (probe == Printer::GetInstance()->m_bed_mesh_probe)
            {
                probe_state_callback_call(probe_data_t{.state = CMD_BEDMESH_PROBE_EXCEPTION});
            }
            throw;
        }
    }
    probe->multi_probe_end();
}
void ProbePointsHelper::_manual_probe_start()
{
    bool done = _move_next(1);
    if (!done)
    {
        std::map<std::string, std::string> params;
        GCodeCommand gcmd = Printer::GetInstance()->m_gcode->create_gcode_command("", "", params);
        ManualProbeHelper(gcmd, std::bind(&ProbePointsHelper::_manual_probe_finalize, this, std::placeholders::_1)); //---??---
    }
}

bool ProbePointsHelper::_fast_probe_start(PrinterProbe *probe, GCodeCommand &gcmd)
{
    std::cout << "_fast_probe_start" << std::endl;
    int loop = 0;
    while (1)
    {
        // 移动到下一点
        bool done = _move_next(2);
        if (done)
        {   
            std::cout << "_fast_probe_start move done............." << std::endl;
            return true; 
        }
        try
        {
            std::vector<double> pos = probe->run_probe(gcmd);
            m_results.push_back(pos);
            probe_data_t probe_event;
            probe_event.state = AUTO_LEVELING_PROBEING;
            probe_event.value = pos[2];
            probe_state_callback_call(probe_event);
            LOG_D("当前触发点的坐标为: %f, %f, %f(%f)\n", pos[0], pos[1], pos[2], pos[3]);
            // TODO: 判断此点坐标与上一次调平点的 差值
            double delta = pos[2] - m_last_fast_probe_points_z.at(loop);
            if ( std::fabs(delta) > probe->m_fast_autoleveling_threshold ) {
                    // 结果清除，重新开始 正常调平
                    m_results.clear();
                    std::cout << "fast auto leveling now:"<< pos[2] << " last: " << m_last_fast_probe_points_z.at(loop) << " delta:"<< delta  << " fast_autoleveling_threshold:" << probe->m_fast_autoleveling_threshold << std::endl;
                    return false;
            }else {
                std::cout << "第 "<< loop <<  " fast auto leveling ok, " << " last: " << m_last_fast_probe_points_z.at(loop) << " delta:"<< delta << " fast_autoleveling_threshold:" << probe->m_fast_autoleveling_threshold << std::endl;
                std::cout << "next point " << std::endl;
            }
        }
        catch (std::exception &e)
        {
            LOG_E("bed mesh probe error\n");
            if (probe == Printer::GetInstance()->m_bed_mesh_probe)
            {
                probe_state_callback_call(probe_data_t{.state = CMD_BEDMESH_PROBE_EXCEPTION});
            }
            return false; 
        }
        loop++;
    }
    return false; 
}

void ProbePointsHelper::_manual_probe_finalize(std::vector<double> kin_pos)
{
    if (kin_pos.size() == 0)
    {
        return;
    }
    m_results.push_back(kin_pos);
    _manual_probe_start();
}

void ProbePointsHelper::_fast_probe_finalize(std::vector<double> kin_pos)
{
}

int probe_register_state_callback(probe_state_callback_t state_callback)
{
    for (int i = 0; i < PROBE_STATE_CALLBACK_SIZE; i++)
    {
        if (probe_state_callback[i] == NULL)
        {
            probe_state_callback[i] = state_callback;
            return 0;
        }
    }
    return -1;
}
int probe_unregister_state_callback(probe_state_callback_t state_callback)
{
    for (int i = 0; i < PROBE_STATE_CALLBACK_SIZE; i++)
    {
        if (probe_state_callback[i] == state_callback)
        {
            probe_state_callback[i] = NULL;
            return 0;
        }
    }
    return -1;
}
int probe_state_callback_call(probe_data_t state)
{
    for (int i = 0; i < PROBE_STATE_CALLBACK_SIZE; i++)
    {
        if (probe_state_callback[i] != NULL)
        {
            probe_state_callback[i](state);
        }
    }
    return 0;
}