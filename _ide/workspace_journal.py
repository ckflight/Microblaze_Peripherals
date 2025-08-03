# 2025-08-03T18:22:18.653270
import vitis

client = vitis.create_client()
client.set_workspace(path="Microblaze_Peripherals")

platform = client.get_component(name="Microblaze")
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

status = platform.update_hw(hw_design = "$COMPONENT_LOCATION/../../../FPGA_Workspace/VIVADO_PROJECTS/XC7A100T_Microblaze/top_module.xsa")

status = platform.build()

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

