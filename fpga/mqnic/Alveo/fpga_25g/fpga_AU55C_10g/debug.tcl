# When debug_clk is set, this script is sourced from compile.tcl to insert a
# Chipscope debug core.

create_debug_core u_ila_0 ila
set_property C_DATA_DEPTH         8192 [get_debug_cores u_ila_0]
set_property C_TRIGIN_EN         FALSE [get_debug_cores u_ila_0]
set_property C_TRIGOUT_EN        FALSE [get_debug_cores u_ila_0]
set_property C_ADV_TRIGGER       FALSE [get_debug_cores u_ila_0]
set_property C_INPUT_PIPE_STAGES     3 [get_debug_cores u_ila_0]
set_property C_EN_STRG_QUAL      FALSE [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU    TRUE [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU_CNT   1 [get_debug_cores u_ila_0]

set debug_clk pcie_user_clk

set_property port_width 1 [get_debug_ports u_ila_0/clk]
connect_debug_port u_ila_0/clk [get_nets $debug_clk]

set debug_nets [lsort -dictionary [get_nets -hier -filter {mark_debug}]]
set n_nets [llength $debug_nets]

if {$n_nets >= 128} {
    puts "WARNING: over 50% of available debug signals used ($n_nets/256), consider removing some debug markings."
}

set_property port_width $n_nets [get_debug_ports u_ila_0/probe0]
connect_debug_port u_ila_0/probe0 $debug_nets
