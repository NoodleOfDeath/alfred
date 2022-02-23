from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
import rospy
from batcomputer.msg import Float32List

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
pub = rospy.Publisher('batcomputer', Float32List, queue_size=1000);


@app.get("/")
async def get_root(request: Request):
	print(request.query_params)
	return "{}"
	
	
@app.get("/status/")
async def get_status(request: Request):
	print(request.query_params)
	return "{}"
	
	
@app.post("/drive/")
async def drive(request: Request):
	json = await request.json()
	speed = json.get('speed', 0.0)
	msg = Float32List()
	msg.data = [1, speed]
	#print(msg)
	pub.publish(msg)
	return "{}"
	
	
@app.post("/rotate/")
async def turn(request: Request):
	json = await request.json()
	speed = json.get('speed', 0.0)
	msg = Float32List()
	msg.data = [2, speed]
	#print(msg)
	pub.publish(msg)
	return "{}"


