# CppRobot

A small side project working with C++ that implements the same behavior found in [a teammate's learning code](https://github.com/chappelleandy/AndrewRobot).

We decided to go with Java instead of C++ for the 2019 season, so this repository is essentially useless. The fact of the matter is that the C++ toolchain, GradleRIO, and working with WPILib for C++ is a major hassle. Sure, arguments can be made that no real-world embedded system will ever use Java, but even WPILib is miles away from the low-level hardware logic that is being done anyways. The API for both languages look the same, and nothing is gained out of using C++ except maybe a few nanoseconds not having to go through the JNI/JNA (not sure which one WPI uses).

That being said, I did learn a number of things about C++ though this repo: object initialization, inheritance patterns, working with a stupid toolchain, etc.

# Credits

Built with [CLion](https://www.jetbrains.com/clion/)

Uses [GradleRIO](https://github.com/Open-RIO/GradleRIO)
