HARDIMAN-X System Specification Document (SSD) v3.1
Tethered Full-Body Hydraulic Load-Carrying Exoskeleton for Nuclear Decommissioning
Author: Sheldon K. Salmon (ORCID: 0009-0005-8057-5115)
Version: v3.1 (November 14, 2026)
License: Creative Commons Attribution 4.0 International (CC BY 4.0)
DOI: 10.5281/zenodo.xxxxxxx (Replace with your reserved Zenodo DOI)
⚠️ IMPORTANT DISCLAIMER
NO HARDWARE EXISTS. This repository contains a concept-level system specification document, not a manufacturing blueprint. Every performance value is a design baseline until physically verified under §13 of the specification. A decided value is not a verified one. This artifact is published as a methodological demonstration of rigorous systems architecture under deep uncertainty, not as evidence of a working system.
📖 About This Project
HARDIMAN-X is a conceptual design for a tethered, legged, full-body hydraulic exoskeleton intended for heavy payload handling in nuclear decommissioning environments.
This repository was built in one week using free-tier AI strictly as a computational and literature-retrieval engine, directed by adversarial red-teaming frameworks and strict epistemic discipline. The primary goal of this project is not to deliver a ready-to-build machine, but to demonstrate how to structure complex system logic, manage uncertainty, and design for failure from day one.
The specification is grounded against the historical General Electric Hardiman I final report [GE-71] and the NASA teleoperator survey [NASA-68], while explicitly addressing the documented failure modes of those heritage systems.
🛡️ Key Architectural Features
This specification distinguishes itself through structural integrity and epistemic honesty:
Epistemic Tagging System: Every numerical claim is tagged by its evidence base ([D] directly confirmed, [R] reasoned, [S] structurally inferred, [?R] extrapolated, [SPEC] architectural proposal). This prevents AI hallucinations or unverified assumptions from being treated as facts.
Physics-Gated Configuration (GVWR Rule): The machine uses a Gross Vehicle Weight Rating-style derating rule. Adding option packs mathematically reduces the rated payload one-for-one, preventing feature creep from silently breaking the machine's physical limits.
First-Principles Rejections: "Cool but dangerous" concepts (e.g., pyrotechnic egress, joint-level micro-surge accumulators, active kinetic ROPS) are explicitly rejected with first-principles math showing exactly how they would cause catastrophic failure or violate safety allocations.
Full Traceability: The document includes a compliance matrix, a parameter register, an assumption map, and a safety-claim register that explicitly lists claims that have not yet been analyzed.
Adversarial Fallbacks: Explicit trigger gates define exactly when to abandon the baseline legged design in favor of the tracked variant (TRK).

📂 Repository Structure

.
├── HARDIMAN-X_SSD_v3.1.pdf          # The compiled, publication-ready specification document
├── HARDIMAN-X_SSD_v3.1.md           # The raw Markdown source of the specification
├── hmx_config_check.py              # Companion configuration model generator (Required by §0)
├── README.md                        # This file
└── LICENSE                          # CC BY 4.0 License

⚙️ Configuration Checker (hmx_config_check.py)
Per §0 Document Conventions of the specification, the configuration model must be re-run after any change to a pack, mass, or position. This script validates the Appendix F tables (Configuration Matrix and Option Packs) and ensures the GVWR derating rules and admissibility checks (CAR-1 to CAR-9) are mathematically sound.
Usage

# Ensure you have Python 3.x installed
python3 hmx_config_check.py

# The script will output a version hash, timestamp, and validate the 
# mass, rating, and offset columns for the defined builds.


Note: The script's version hash and the date of its last run must be published with each release of this specification.
🔍 Current Limitations & Open Items
As a concept-level baseline, this document openly acknowledges its evidence gaps. Per §15 Open Items and Appendix G:
Of the 25 source checks in Appendix C.1, 17 are not yet independently verified.
30 external evidence requests (EXT-01 to EXT-30) are open for deep search or physical testing.
The safety-claim register (HMX-10.1-04) holds 4 claims that have not yet been analyzed via FMEA.
No CAD mass properties or physical hardware-limb test data exist yet (HMX-13.2-09).
The next phase of work is not writing more requirements, but closing these evidence gaps through targeted literature review, supplier data acquisition, and eventual hardware mockup testing.
📚 Citation
If you use this specification or its methodology in your research, please cite it as:
Salmon, S. K. (2026). HARDIMAN-X System Specification Document (SSD) v3.1: Tethered Full-Body Hydraulic Load-Carrying Exoskeleton for Nuclear Decommissioning. Zenodo. https://doi.org/10.5281/zenodo.xxxxxxx
🤝 Engagement & Red-Teaming
This document is designed to invite scrutiny. If you are a systems engineer, robotics architect, or safety analyst, your feedback is highly valued.
How to engage:
Open an Issue to point out structural failure nodes, undefined terms, or exploitable loopholes in the logic.
Submit a Pull Request with verified data to close any of the EXT-xx items in Appendix G.
Critique the epistemic tagging or the GVWR derating mathematics.
Final judgment on load-bearing elements remains with the architect, but adversarial audits are the primary mechanism for improving this specification.
📬 Contact
GitHub: github.com/AionSystem
ORCID: 0009-0005-8057-5115
