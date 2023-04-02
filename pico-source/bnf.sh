cowsay Give em the ol build n flash

# Build
cd build
make -j4
cd ..

# Flash
sudo picotool load build/src/controller.elf
sudo picotool reboot
