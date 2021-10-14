# 3. Semsterprojekt
How to get this project to work for Linux users:

1. Clone (download) the project to a folder somewhere (fx /home/username/Documents/Github/)
2. Enter the build folder of which ever project you want to work on (kinematics or Machine-Vision)
3. Call "sudo cmake ../src/" from terminal. Make sure the terminal is called from inside the build-folder (you can instead call "sudo cmake -DCMAKE_BUILD_TYPE=Debug ../src/", i think its better for debugging)
4. Open Qt Creator and press the "Open" project button
5. Navigate to the project and in the "src" folder choose "CMakeLists.txt". (fx /home/username/Documents/Github/3.-Semsterprojekt/kinematics/src/CMakeLists.txt)
6. Accept the auto selected Debug kit
7. Congratz you can now work on the project :)
