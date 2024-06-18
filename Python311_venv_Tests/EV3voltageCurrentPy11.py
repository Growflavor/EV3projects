#!/home/robot/py11venv/bin/python3.11

with open('/sys/class/power_supply/lego-ev3-battery/voltage_now') as voltage_file:
    voltage = int(voltage_file.read())
    print("Current voltage: {} µV".format(voltage))

# Current voltage: 8267133µV

with open('/sys/class/power_supply/lego-ev3-battery/current_now') as current_file:
    current = int(current_file.read())
    print("Current current: {} µA".format(current))

# Current current: 149333µA