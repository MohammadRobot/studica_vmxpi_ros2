# Security policy

This repository is pre-production. No release is currently approved for an
untrusted network or unattended remote motion.

Do not report a suspected vulnerability in a public issue. Use GitHub's private
vulnerability-reporting or security-advisory feature for this repository. If
that is unavailable, contact the maintainer listed in `package.xml` and include
only the minimum information needed to establish a private channel.

Include the affected commit or version, robot/OS model, impact, reproduction
conditions, and whether physical motion or credentials are involved. Do not
include passwords, private keys, Wi-Fi credentials, or personal data.

Security fixes are coordinated privately until a patch, affected-version list,
upgrade instructions, and disclosure date are ready. Safety-impacting reports
receive the same handling as security reports.

Current hardening requirements include unique first-boot credentials, SSH keys,
least-privilege services, SROS2/DDS permissions, signed artifacts, rollback,
and no direct DDS exposure to the internet.
