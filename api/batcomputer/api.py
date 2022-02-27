from curses import raw
import json
import rospy
import sqlite3
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from std_msgs.msg import Float32
from batcomputer.msg import CreateCommand

app = FastAPI()

origins = ["*"]

# Enable cross origins
app.add_middleware(
  CORSMiddleware,
  allow_origins=origins,
  allow_credentials=True,
  allow_methods=["*"],
  allow_headers=["*"],
)

rospy.init_node('batcomputer')
pub = rospy.Publisher('/batcomputer', CreateCommand, queue_size=1000);

def run_query(query: str):
  con = sqlite3.connect('status.db')
  cursor = con.cursor()
  cursor.execute(query)
  result = cursor.fetchall()
  con.close()
  return result
  
"""
CREATE TABLE sensor_states (
  topic       VARCHAR (255) PRIMARY KEY NOT NULL,
  value       VARCHAR (255) NOT NULL,
  timestamp   DATETIME DEFAULT CURRENT_TIMESTAMP
);
"""

@app.get("/")
async def get_root(request: Request):
  return "{}"
  
  
@app.get("/status/")
async def get_status(request: Request):
  record = {}
  try:
    raw_records = run_query(f"SELECT * FROM sensor_states")
    if len(raw_records) < 1:
      return json.dumps(record)
    for raw_record in raw_records:
      record[raw_record[0]] = raw_record[1]
  except Exception as e:
    print(e)
  return json.dumps(record)
  
@app.post("/drive/")
async def drive(request: Request):
  json = await request.json()
  velocity = json.get('velocity', 0.0)
  msg = CreateCommand()
  msg.command = CreateCommand.CMD_DRIVE
  msg.velocity = velocity
  #print(msg)
  pub.publish(msg)
  return "{}"
  
  
@app.post("/rotate/")
async def turn(request: Request):
  json = await request.json()
  velocity = json.get('velocity', 0.0)
  msg = CreateCommand()
  msg.command = CreateCommand.CMD_ROTATE
  msg.velocity = velocity
  #print(msg)
  pub.publish(msg)
  return "{}";
  

  
  
