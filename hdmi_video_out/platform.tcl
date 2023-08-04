# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct /home/antonio/git/zybo-usb/hdmi_video_out/platform.tcl
# 
# OR launch xsct and run below command.
# source /home/antonio/git/zybo-usb/hdmi_video_out/platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {hdmi_video_out}\
-hw {/home/antonio/git/hdmi_video_out/hdmi_video_out.xsa}\
-out {/home/antonio/git/zybo-usb}

platform write
domain create -name {standalone_ps7_cortexa9_0} -display-name {standalone_ps7_cortexa9_0} -os {standalone} -proc {ps7_cortexa9_0} -runtime {cpp} -arch {32-bit} -support-app {hello_world}
platform generate -domains 
platform active {hdmi_video_out}
domain active {zynq_fsbl}
domain active {standalone_ps7_cortexa9_0}
platform generate -quick
platform generate
bsp reload
bsp reload
domain active {zynq_fsbl}
bsp reload
platform generate -domains 
platform generate
platform clean
platform generate
platform generate
platform generate
platform active {hdmi_video_out}
domain active {standalone_ps7_cortexa9_0}
bsp reload
platform generate -domains 
platform generate
platform generate
platform generate
platform generate
platform generate
