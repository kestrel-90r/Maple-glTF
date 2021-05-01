# Maple-glTF
 The name Maple is a pun for MAP and Light Editing. I wish I could describe 3D space very easily and program various things.
![screenshot](https://github.com/itakawa/Maple-glTF/blob/main/ss.png?raw=true)

# Required environment
Microsoft Visual Studio 2019 Version 16.8.2

# How to installation, and building

Create a new folder that is not Japanese,Then open a prompt and, Get the set of project files with the following command.
*If you install it in a Japanese folder, it will fail to browse the resource files.

```
git clone https://github.com/itakawa/Maple-glTF.git
cd Maple-glTF
```

Open the solution file(testMapleGLTF.sln)

Select the executable file format from Debug to Release and from x64 to x86.

*The old Siv3D (August2016v2) is configured for x86, so it will not work on x64.

*Also, open the project properties page and select Visual Studio 2015 (V140) in the Platform Toolset item in the General menu.
If the installation is not complete, you need to install the toolset in advance.

