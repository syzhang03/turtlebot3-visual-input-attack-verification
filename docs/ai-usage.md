# AI assistance and human validation

Codex assisted with the initial ROS package, HSV detector, control state machine, configuration, obstacle SDF, tests, shell helpers, CSV summarizer and documentation. The initial source was generated in a Windows workspace and transferred to Ubuntu for execution.

Wu Haonan manually operated the Ubuntu virtual machine, verified the environment, ran the tests and Gazebo trials, investigated rendering/display issues with assistance, and reported results and physical no-contact observations in the collaboration conversation. The contributor supplied a transcript with 10 passing unit tests. Repository evidence should be checked against the original per-run files rather than relying on this narrative alone.

Codex subsequently prepared this submission and evidence index. Any machine-readable summaries recomputed from CSV files are identified in the results index. No group member's review or approval is asserted; a teammate should review the implementation and reproduction instructions before merging.

For the delay-replay stage, Codex assisted with the bounded historical-frame relay, reliable output transport, scene launch helpers, optional endpoint stopping, tests and offline log auditing. Wu Haonan ran the Ubuntu/Gazebo experiments and reported contact or no-contact observations. The 2026-10-30 evidence index separates raw logs, recomputed results, operator reports and console-only records; screenshots reported as saved but not supplied are not presented as repository evidence.

Known remaining work: independent ROS integration validation by another member, same-version repeated controls, automated contact/clearance measurements, LiDAR mitigation and formal controlled comparisons. Project claims must remain limited to the tests actually performed.
