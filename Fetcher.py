from peewee import *
from playhouse.sqlite_ext import SqliteExtDatabase
import datetime

db = SqliteExtDatabase('datalog.db')
port = 'sample/port123'

class BaseModel(Model):
    class Meta:
        database = db

class Telemetry(BaseModel):
    latitude = CharField()
    longitude = CharField()
    satellites = IntegerField()
    speed = DecimalField()
    altitude = DecimalField()
    acc_x = DecimalField()
    acc_y = DecimalField()
    acc_z = DecimalField()
    temperature = DecimalField()
    voltage = DecimalField(default=3.3)
    current = DecimalField()
    is_separated = BooleanField()
    light_input = DecimalField()
    timestamp = CharField()

db.connect()
# If we don't have the table yet, create it
if not Telemetry.table_exists():
    Telemetry.create_table(True)

# Keep trying to connect until it is successful
while True:
	try:
	    xbee = serial.Serial(port)
	    break
	except:
	    pass

# Continuously fetch data
while True:
	data = self.xbee.read()
        data.split(",")

	# The exact format of the data and how it will be added to the sqlite database is not specified here
	Telemetry.create(
                current=data[0], 
                is_separated=data[1], 
                altitude=data[2], 
                temperature=data[3], 
                acc_x=data[4], 
                acc_y=data[5], 
                acc_z=data[6],
                timestamp=data[7],
                latitude=data[10] if len(data) > 10 else "",
                longitude=data[11] if len(data) > 11 else "",
                speed=data[12] if len(data) > 12 else 0.0,
                altitude=data[13] if len(data) > 13 else 0.0,
                satellites=data[14] if len(data) > 14 else 0
                )
