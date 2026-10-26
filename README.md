# Reflective Surface Detection & 3D-to-2D Volumetric Mapping

This repository contains the implementations associated with two research works focused on robust mobile robot map building:

---

## 📁 Repository Structure

1. **`glassDetection/`**
   * **Description:** Analyzes laser scanner intensity readings to identify highly reflective surfaces (such as glass). It modifies the laser scans accordingly to ensure robust occupancy grid map generation in challenging environments.
   * **Associated Paper:** *Intensity-based identification of reflective surfaces for occupancy grid map modification* (IEEE Access, 2023).

2. **`volumeMapping/`**
   * **Description:** Utilizes probabilistic treatment to accumulate 3D LiDAR/sensor scans and generate a 2D occupancy map where the resulting map captures the full volumetric shape of objects.
   * **Associated Paper:** *Leveraging 3-D data for whole object shape and reflection aware 2-D map building* (IEEE Sensors Journal, 2023).

---

## 📄 Citation

If you use this code or work in your research, projects, or teaching, please cite the corresponding papers:

```bibtex
@article{mora2023intensity,
  title     = {Intensity-based identification of reflective surfaces for occupancy grid map modification},
  author    = {Mora, Alicia and Prados, Adri{\'a}n and Gonz{\'a}lez, Pavel and Moreno, Luis and Barber, Ram{\'o}n},
  journal   = {IEEE Access},
  volume    = {11},
  pages     = {23517--23530},
  year      = {2023},
  publisher = {IEEE}
}

@article{mora2023leveraging,
  title     = {Leveraging 3-D data for whole object shape and reflection aware 2-D map building},
  author    = {Mora, Alicia and Barber, Ramon and Moreno, Luis},
  journal   = {IEEE Sensors Journal},
  volume    = {24},
  number    = {14},
  pages     = {21941--21948},
  year      = {2023},
  publisher = {IEEE}
}
