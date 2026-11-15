# Pipeline diagnostics checkpoint

Changes after `719c138` add attribution for acquisition loss and scheduling
stalls without increasing queue sizes or changing the timestamp matching policy.

Per-camera reports distinguish SDK receive/skip counters, callback ingress,
driver enqueue/drop counts, queue high-water marks, dequeued callback age,
main-stream publications, preserved unmatched publications, and publication
failures. Unmatched archival publications are not counted as dropped images.
Publication wall time and thread CPU time are recorded separately, helping
distinguish computation from waiting. Lifetime counters and interval peaks
remain distinct; synchronization warmup still emits the statistics.

`spinnaker_camera_driver/test/pipeline_counters_test.cpp` checks SDK versus
driver-drop attribution, unmatched/failure classification, CPU versus wall-time
accounting, and interval-peak reset behavior. Build with `BUILD_TESTING=ON`
and run the package's ament tests; the header-only tests can also be compiled
with GoogleTest independently of camera hardware. These counters diagnose
driver boundaries; they do not alone prove end-to-end recorder completeness.
