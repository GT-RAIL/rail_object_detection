^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rail_object_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.2 (2018-11-01)
------------------
* Update the default branch

3.0.1 (2018-09-07)
------------------
* Create a separate package for the messages
* Contributors: Siddhartha Banerjee, banerjs

2.0.1 (2017-12-11)
------------------
* Added Deformable RFCN detector.
* Change darknet node name.
* Change directory structure and file names for darknet.
* Update README with valid weights file, and fix broken links
* Contributors: Andrew Silva, Ryan Petschek, Siddhartha Banerjee, Weiyu Liu

1.0.4 (2017-02-09)
------------------
* Adding in an automatic build for 32 bit
* Contributors: Siddhartha Banerjee, banerjs

1.0.3 (2017-02-09)
------------------
* Adding in base travis build
* Fixed the 32-bit compile error, hopefully
* Contributors: Siddhartha Banerjee

1.0.2 (2017-02-03)
------------------
* Completed the build of GPU with flags
* Pushing fixes in master back to gpu_devel. Merge branch 'master' into gpu_devel
* Updated the README. Unfortunately, the mangling of data between publisher and service still exists and I cannot get rid of it
* Fixed timing bugs with the object detector and possibly even the bug between service and topic contention. Need to test
* Merging updates on master into the GPU branch
* Created and Tested install of the package
* Removed the need to update internal config file values
* Returning defaults
* Changed naming for a public release.
* Testing out GPU functionality
* Adding in a gitignore
* Updated the detector to also use a topic for publishing object detections
* removed unused parameters
* removed service name parameter
* Updated path in test script to use package-based absolute path
* package-location-based absolute paths for ros params, minor cleanup and debugging, documentation
* Adding in the cfg and data folders
* Completed final details on object detection. Delivering project for now
* Cursory stress test of the node is complete
* Finally done with a functioning prototype of the object detector
* Successful linkage of C++ with C
* Completed ROS Skeleton for the detector
* Initial commit of the object detector
* Contributors: David Kent, Siddhartha Banerjee
