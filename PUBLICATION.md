# Sanitized source publication — 2026-10-27

This distribution is prepared from the archived Agribot repositories. The original archives remain unchanged outside the publication copies.

- Known credentials and private service-account files have been removed from the published reachable Git history. Configuration examples require credentials supplied locally.
- Internal Agribot submodule URLs point to https://github.com/panpawelchmiel/. Gitlinks are mapped to the corresponding sanitized commits; publication-snapshot tags keep the referenced source revisions available.
- Apache-2.0 applies to the project contributions; original third-party license terms and attribution remain effective. See THIRD_PARTY_NOTICES.md.
- This source publication does not certify an NCBR deliverable scope or a successful robot/hardware build. Vendor packages, external submodules and runtime credentials may need to be obtained separately.
- Source commits retain their original authors. Additional publication commits describe sanitization/licensing/configuration changes. Historic commit IDs can change as a result of sanitization.

## Local configuration

Do not commit live credentials. Supply AWS_ACCESS_KEY_ID and AWS_SECRET_ACCESS_KEY locally when using the Docker Compose services. For agribot_cloud copy the .yaml.example files to their .yaml names and configure service tokens locally. For Android obtain your own app/google-services.json from the appropriate Firebase project; do not use or commit an Admin SDK private key. Configure Wi-Fi credentials before using hostapd. Review all examples before deployment.

The sentinel AGRIBOT_CONFIGURE_LOCALLY in historical code identifies a credential removed during publication. It is not a working token.
