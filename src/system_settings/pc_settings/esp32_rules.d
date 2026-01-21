# esp32 for Left Leg - Plug into specific port left side of the pc
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7522", KERNELS=="1-9", SYMLINK+="biped_esp32_left"

# Right Leg - Plug into specific port right side of the pc 
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7522", KERNELS=="1-1", SYMLINK+="biped_esp32_righht"