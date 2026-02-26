To install dependencies, use [Conda](https://docs.conda.io/en/latest/):

`conda env create -f environment.yaml`
`conda activate pump_control env`

To manually run the GUI from python, run
`python pump_sensor_gui.py`

To compile into an executable, run
`pyinstaller --onefile --windowed pump_sensor_gui.py`

Logs will be output by default to the `logs` folder.