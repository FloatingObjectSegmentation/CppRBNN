## CppRBNN

This is a fast implementation of RBNN in C++. Works only on Windows 10.

## Building from source

- Install PCL 1.8.1
- Install cmake 3.0 or higher
- Build using cmake from the root folder using CMakeLists.txt and the main .cpp file
- Open the project in Visual Studio 2017
- Build in Release mode

## Usage

- After building the project in release mode, ```rbnn.exe``` can be found
  in the release folder. It is a command line tool with the following usage:
    - .\rbnn.exe <path_to_directory> <xyz_pcd_file> <radius_values_delimited_by_space>
    - Radius values determine for which radius thresholds you want rbnn to
      execute and save the results in the results file.
    - **example**: ```.\rbnn.exe C:\myfolder file.pcd 0.5 1 3 5 10```

## Result

After ```rbnn``` is finished executing, it will output a result file in the same folder.
This file has a line for each radius value. The values are separated by a space. The first
value is the radius threshold of rbnn. Following are the classifications of each of the
points in the input point cloud according to RBNN. -1 represents the class of points that
belong to the floor. All other classes belong to non-floor points, represented with 
non-negative integers. Those points that have the same integer value belong to the same
segment. Here is an example...

```
3 -1 -1 -1 -1 -1 1 1 1 -1 -1 -1 -1 2 2 2 2
5 -1 -1 -1 -1.... 123 123 123 123 -1 -1 532 532 532 ...
```

