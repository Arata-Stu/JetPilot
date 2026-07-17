# jetpilot_control (legacy bench publisher)

This package publishes a fixed `/auto/control_cmd` for bench checks. It is no longer included by
`jetpilot_system_launch`; autonomous driving uses `jetpilot_controller` and its Pure Pursuit input
watchdogs. Do not launch both packages together because they publish the same command topic.
