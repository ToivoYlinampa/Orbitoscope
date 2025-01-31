# Orbitoscope
A six-axis macro-imaging robot to capture EDOF images for photogrammetrical use

Developed in The University of Tartu / The Natural History Museum of Tartu, as a part of my Ph.D. studies. Paper coming soon.

## License  
This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for details.  

**Important Note:**  
This software relies on `gphoto2` (command-line tool), which is licensed under the **GNU GPL**.  
- `gphoto2` is **not included** in this repository. Users must install it separately and comply with its [GPL terms](https://www.gnu.org/licenses/gpl-3.0.html).  
- Your use of `gphoto2` (via CLI) does not affect the MIT licensing of *this codebase*.  

## Third-Party Dependencies  
This project uses the following libraries:  
- [OpenCV](https://opencv.org/) (Apache 2.0)  
- [PySerial](https://pythonhosted.org/pyserial/) (BSD 3-Clause)  
- [NumPy](https://numpy.org/) (BSD 3-Clause)  
- Python Standard Libraries (e.g., `csv`, `datetime`, `subprocess`) (PSF License)  

Full license texts for dependencies are provided in [LICENSE-3RD-PARTY](LICENSE-3RD-PARTY).
