# 2025-08-03T01:12:20.915078
import vitis

client = vitis.create_client()
client.set_workspace(path="Microblaze_Peripherals")

advanced_options = client.create_advanced_options_dict(dt_overlay="0")

platform = client.create_platform_component(name = "Microblaze",hw_design = "$COMPONENT_LOCATION/../../../FPGA_Workspace/VIVADO_PROJECTS/XC7A100T_Microblaze/top_module.xsa",os = "standalone",cpu = "microblaze_0",domain_name = "standalone_microblaze_0",generate_dtb = False,advanced_options = advanced_options,compiler = "gcc")

platform = client.get_component(name="Microblaze")
domain = platform.get_domain(name="standalone_microblaze_0")

status = domain.set_config(option = "lib", param = "XILTIMER_en_interval_timer", value = "true", lib_name="xiltimer")

comp = client.create_app_component(name="Microblaze_Peripherals",platform = "$COMPONENT_LOCATION/../Microblaze/export/Microblaze/Microblaze.xpfm",domain = "standalone_microblaze_0",template = "hello_world")

status = platform.build()

status = platform.build()

comp = client.get_component(name="Microblaze_Peripherals")
comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

