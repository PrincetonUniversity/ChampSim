#include "options.h"

namespace champsim
{

void Options::set(environment& env)
{
  for (O3_CPU& cpu : env.cpu_view()) {
    cpu.resize_cpu();
  }
}

void Options::init(CLI::App& app)
{
  // CPU parameters
  app.add_option("--ifetch-buffer-size", ifetch_buffer_size, "The size of the instruction fetch buffer");
  app.add_option("--dispatch-buffer-size", dispatch_buffer_size, "The size of the dispatch buffer");
  app.add_option("--decode-buffer-size", decode_buffer_size, "The size of the decode buffer");
  app.add_option("--rob-size", rob_size, "The size of the reorder buffer");
  app.add_option("--sq-size", sq_size, "The size of the store queue");

  app.add_option("--fetch-width", fetch_width, "The width of the instruction fetch unit");
  app.add_option("--decode-width", decode_width, "The width of the decode unit");
  app.add_option("--dispatch-width", dispatch_width, "The width of the dispatch unit");
  app.add_option("--scheduler-size", scheduler_size, "The size of the scheduler");
  app.add_option("--exec-width", exec_width, "The width of the execution unit");

  app.add_option("--lq-width", lq_width, "The width of the load queue");
  app.add_option("--sq-width", sq_width, "The width of the store queue");

  app.add_option("--retire-width", retire_width, "The width of the retire unit");

  app.add_option("--branch-mispredict-penalty", branch_mispredict_penalty, "The penalty for a branch misprediction");
  app.add_option("--dispatch-latency", dispatch_latency, "The latency of the dispatch unit");
  app.add_option("--decode-latency", decode_latency, "The latency of the decode unit");
  app.add_option("--scheduling-latency", scheduling_latency, "The latency of the scheduler");
  app.add_option("--exec-latency", exec_latency, "The latency of the execution unit");

  app.add_option("--l1i-bandwidth", l1i_bandwidth, "The bandwidth of the L1 instruction cache");
  app.add_option("--l1d-bandwidth", l1d_bandwidth, "The bandwidth of the L1 data cache");
}

void Options::update(environment& env)
{
  for (O3_CPU& cpu : env.cpu_view()) {
    if (ifetch_buffer_size)
      cpu.IFETCH_BUFFER_SIZE = ifetch_buffer_size;
    if (dispatch_buffer_size)
      cpu.DISPATCH_BUFFER_SIZE = dispatch_buffer_size;
    if (decode_buffer_size)
      cpu.DECODE_BUFFER_SIZE = decode_buffer_size;
    if (rob_size)
      cpu.ROB_SIZE = rob_size;
    if (sq_size)
      cpu.SQ_SIZE = sq_size;

    if (fetch_width)
      cpu.FETCH_WIDTH = fetch_width;
    if (decode_width)
      cpu.DECODE_WIDTH = decode_width;
    if (dispatch_width)
      cpu.DISPATCH_WIDTH = dispatch_width;
    if (scheduler_size)
      cpu.SCHEDULER_SIZE = scheduler_size;
    if (exec_width)
      cpu.EXEC_WIDTH = exec_width;

    if (lq_width)
      cpu.LQ_WIDTH = lq_width;
    if (sq_width)
      cpu.SQ_WIDTH = sq_width;

    if (retire_width)
      cpu.RETIRE_WIDTH = retire_width;

    if (branch_mispredict_penalty)
      cpu.BRANCH_MISPREDICT_PENALTY = branch_mispredict_penalty;
    if (dispatch_latency)
      cpu.DISPATCH_LATENCY = dispatch_latency;
    if (decode_latency)
      cpu.DECODE_LATENCY = decode_latency;
    if (scheduling_latency)
      cpu.SCHEDULING_LATENCY = scheduling_latency;
    if (exec_latency)
      cpu.EXEC_LATENCY = exec_latency;

    if (l1i_bandwidth)
      cpu.L1I_BANDWIDTH = l1i_bandwidth;
    if (l1d_bandwidth)
      cpu.L1D_BANDWIDTH = l1d_bandwidth;
  }

  set(env);
}

} // namespace champsim