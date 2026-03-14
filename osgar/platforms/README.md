# OSGAR Platforms

This directory contains drivers and configuration for various robotic platforms supported by OSGAR.

## Supported Robots

### Matty
*   **Description:** 4-wheel drive articulated robot with a passive joint.
*   **Driver:** `osgar.platforms.matty:Matty`
*   **Links:** [Technical details](https://robotika.vosrk.cz/robots/matty/), [Matty Twins](https://robotika.cz/robots/matty-twins/)

### Eduro
*   **Description:** Rugged 4-wheel drive robot used in various outdoor competitions.
*   **Driver:** `osgar.platforms.eduro:Eduro`

### Kloubak
*   **Description:** Articulated robot developed at CZU (Czech University of Life Sciences Prague).
*   **Driver:** `osgar.platforms.kloubak:RobotKloubak`
*   **Repository:** [tf-czu/kloubak](https://github.com/tf-czu/kloubak)

### Spider
*   **Description:** Spider3 Rider platform, often used with Velodyne and GPS for autonomous navigation.
*   **Driver:** `osgar.platforms.spider:Spider`

### Maria
*   **Description:** A small tank-like robot with a text-based communication protocol.
*   **Driver:** `osgar.platforms.maria:RobotMaria`

### Cortexpilot (Robik, Skiddy)
*   **Description:** Driver for robots from cortexpilot.com, including Robik and Skiddy.
*   **Driver:** `osgar.platforms.cortexpilot:Cortexpilot`

### Deedee
*   **Description:** Delivery-style robot with SLIP-encoded communication.
*   **Driver:** `osgar.platforms.deedee:Deedee`

### FR07 (Yuhesen)
*   **Description:** Commercial outdoor delivery platform FR-07 Pro from Yuhesen.
*   **Driver:** `osgar.platforms.yuhesen:FR07`
