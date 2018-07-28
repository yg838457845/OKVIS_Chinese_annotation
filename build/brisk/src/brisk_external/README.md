# QUICKSTART #

This is the Author's implementation of 
BRISK: Binary Robust Invariant Scalable Keypoints [1]. 
Various (partly unpublished) extensions are provided, some of which are 
described in [2].

 [1] Stefan Leutenegger, Margarita Chli and Roland Siegwart. BRISK:
 Binary Robust Invariant Scalable Keypoints, in Proceedings of
 the IEEE International Conference on Computer Vision (ICCV2011).
 
 [2] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms 
 for Efficient and Robust Autonomous Operation. Doctoral dissertation, 2014.
 
Note that the codebase that you are provided here is free of charge and without 
any warranty.This is bleeding edge research software.
 
### License ###

The 3-clause BSD license (see file LICENSE) applies.

### How do I get set up? ###

Supported operating systems: Linux or MacOS X, tested on Ubuntu 14.04 and El
Capitan. Vector instructions (SSE2 and SSSE3 or NEON) must be available.

Dependencies: 

* OpenCV 2.4 or newer. OpenCV 3 is compatible, however not extensively tested
  and the demo application is somewhat limited in functionality.

Build instructions:
```terminal
cd /path/to/brisk
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
```
Run the demo application as
```terminal
bin/demo
```
There are various options for the different versions of BRISK and other 
feature's detection and description.
To see them, run
```terminal
bin/demo --help
```

### Using BRISK in your application ###

The demo.cc should give you enough details about how to use BRISK. 
We recommend the following setting:

* Detector: brisk::BriskFeatureDetector
* Descriptor: brisk::BriskDescriptorExtractor 
  (brisk::BriskDescriptorExtractor::Version::briskV2)
* Matcher: brisk::BruteForceMatcher

Note about invariances: only use the ones you need specific to your application. 
Invarinaces will always reduce discriminative power.

For visual odometry/SLAM applications, we recommend a different detector that 
makes sure to distribute keypoints homogeneously in the image. Use 
brisk::HarrisScaleSpaceFeatureDetector(threshold, octaves, absoluteThreshold), 
where the threshold is inversely proportional to the feature density in the 
image and the absoluteThreshold should be set higher than 0 (e.g. 100), in 
order to suppress detections from noise in uniform areas.

Warning about the cv::KeyPoint size field: this is used to scale the BRISK 
sampling pattern. So when using non-brisk detectors, make sure this field is
appropriately set. Too small values will lead to very bad descriptor 
performance; too large values, however, will lead to good descriptor 
discriminative power, but also to removal of many  keypoints near the image
border.

### Contribution guidelines ###

If you would like to become a contributor, please contact 
s.leutenegger@imperial.ac.uk.

### Requests, bug reports ###

Please read this guide carefully first! 
Contact s.leutenegger@imperial.ac.uk. 

