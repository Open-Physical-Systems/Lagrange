# Security policy

## Reporting a vulnerability

Do not open a public issue. Use GitHub's private vulnerability reporting for this repository
("Security" tab, "Report a vulnerability"). We aim to acknowledge a report within 7 days.

## Supported versions

| Version | Fixes |
|---|---|
| 1.x (`release/1.x`) | Security fixes until ROS 2 Lyrical's end of life (May 2031), RFC 0007 |
| 0.x | None: update to 1.x |

## Using Lagrange safely

The viewer protocol listens on a Unix socket (mode 0600) by default. Over a network, use a
`tls://` endpoint with a certificate and an authentication token (`--auth-token-file`); the
server refuses to listen beyond loopback without a token. Plugins and FMUs are programs: load
only those you trust. The threat model, fuzzing and dependency review are in
`docs/design/security.md`.
