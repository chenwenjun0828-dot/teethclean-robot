# Architecture

Registration accepts a camera point cloud and a matching scan mesh. Manual
landmarks or the automatic FPFH/RANSAC/ICP path estimate a scan-to-camera rigid
transform. The workflow combines this with camera pose metadata to produce a
scan-to-world transform in metres, while keeping registration residuals and
input fingerprints in its JSON report.
