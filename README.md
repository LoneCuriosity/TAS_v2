# Telemetry Acquisition System v2
Date: 7/2/2023
## Screenshots

![App Screenshot](https://ramongarciajr.tech/TAS_v2_3.jpeg)


## Lessons Learned

Throughout the design process of TAS v2, I acquired valuable lessons and knowledge in both technical and management aspects. On the technical side, I gained a comprehensive understanding of different protocols, their comparative advantages, and the reasons behind their suitability for specific applications. Additionally, I delved into the intricacies of PCB design and manufacturing, learning about the critical role played by material selection in various practical scenarios. In terms of management, I developed skills in effectively leading and coordinating a large team towards a common objective, while also becoming proficient in setting project goals. This experience has significantly expanded my skill set and deepened my expertise in these domains.


## Optimizations

Although we had envisioned implementing certain optimizations, we encountered limitations that prevented their inclusion in the project. Specifically, two optimizations that we desired were the integration of a different GPS receiver module and a transition from LoRa to HAM. Initially, we utilized the PA1616S GPS module by Adafruit as our primary choice. However, we faced challenges due to its large size and limited compatibility with the GPS constellation alone. Consequently, we aimed to switch to a smaller u-blox GPS receiver, which would offer compatibility with multiple, if not all, constellations. Another optimization involved transitioning from LoRa to HAM. The primary driver for this change was the enhanced range offered by HAM at lower frequencies, surpassing that of LoRa. Additionally, LoRa's proprietary nature restricted our ability to exert fine control over packet transmission and protocol selection, which would not be an issue with HAM.


## Run Locally

Clone the project

```bash
git clone https://github.com/LoneCuriosity/TAS_v2
```

Go to EasyEDA

```bash
https://easyeda.com/editor
```

Go to

```bash
File > Open > EasyEDA
```


## 🛠 Skills
EasyEDA, Solidworks


## Tech Stack

- I2C
- SPI
- UART
- Arduino

## 🔗 Links
[![portfolio](https://img.shields.io/badge/my_portfolio-000?style=for-the-badge&logo=ko-fi&logoColor=white)](https://ramongarciajr.tech/)


## Authors

- [@LoneCursioty](https://www.github.com/LoneCursioty)

