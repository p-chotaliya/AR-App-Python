import cv2
import numpy as np
import dlib
import glob
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math
import sys
import time

glass = glob.glob('*.obj')
glass.sort()
global currentobj
currentobj=0
global iszisy
iszisy=1
global x
global y
global z
global ax
global ay
global az
x=0.0
y=0.0
z=0.0
ax=0.0
ay=0.0
az=0.0

width = 635
height = 480
nRange = 1.0

global capture
capture = None
global ret
ret = None
global detector
detector = dlib.get_frontal_face_detector()
global pedictor
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")
global co
co=0

def sortFirst(val): 
    return val[0]  

def Length(ver):
	minz=ver[0][2]
        maxz=ver[0][2]
        for v in ver:
            if v[2]<minz:
                minz=v[2]
            elif v[2]>maxz:
                maxz=v[2]
        return maxz-minz,minz


class ObjLoader(object):
    def newload(self, fileName):
        self.vertices = list()
        self.faces = list()
        self.texels = list()
        self.normals = list()
        self.mmx=[0,0]
        self.mmy=[0,0]
        self.mmz=[0,0]
	self.mz=-10000
	
        ##
        try:
            file = open(fileName)
            for line in file:
                if line.startswith('v '):
                    line = line.strip().split()
                    vertex = [float(line[1]), float(line[2]), float(line[3]) ]
                    self.vertices.append(vertex)

                elif line.startswith('f '):
                    face=[]
                    line = line.strip().split()
                    for i in range(1,len(line)):
                        l=line[i].strip().split('//')
                        if len(l)==2:
                            v=(int(l[0]),1,int(l[1]))
                            face.append(v)
                        else:
                            l = line[i].strip().split('/')
                            v=(int(l[0]),int(l[1]),int(l[2]))
                            face.append(v)
                    self.faces.append(face)
                elif line.startswith("vt"):
                    line = line.strip().split()
                    texel = (float(line[1]), float(line[2]) )
                    self.texels.append(texel)
                
                elif line.startswith("vn"):
                    line = line.strip().split()
                    normal = (float(line[1]), float(line[2]), float(line[3]) )
                    self.normals.append(normal)
            file.close()
        except IOError:
            print(".obj file not found.")

    def minmax(self):
        self.mmx[0]=self.vertices[0][0]
        self.mmy[0]=self.vertices[0][1]
        self.mmz[0]=self.vertices[0][2]
        self.mmx[1]=self.vertices[0][0]
        self.mmy[1]=self.vertices[0][1]
        self.mmz[1]=self.vertices[0][2]
        for v in self.vertices:
            if v[0]<self.mmx[0]:
                self.mmx[0]=v[0]
            elif v[0]>self.mmx[1]:
                self.mmx[1]=v[0]
            if v[1]<self.mmy[0]:
                self.mmy[0]=v[1]
            elif v[1]>self.mmy[1]:
                self.mmy[1]=v[1]
            if v[2]<self.mmz[0]:
                self.mmz[0]=v[2]
            elif v[2]>self.mmz[1]:
                self.mmz[1]=v[2]
	temp=0.0
	if not(self.mmx[1]-self.mmx[0]>self.mmy[1]-self.mmy[0] and self.mmz[1]-self.mmz[0]>self.mmy[1]-self.mmy[0]):
		for i in range(0,len(self.vertices)):
			temp=self.vertices[i][1]
			self.vertices[i][1]=self.vertices[i][2]
			self.vertices[i][2]=-temp

        self.mmx[0]=self.vertices[0][0]
        self.mmy[0]=self.vertices[0][1]
        self.mmz[0]=self.vertices[0][2]
        self.mmx[1]=self.vertices[0][0]
        self.mmy[1]=self.vertices[0][1]
        self.mmz[1]=self.vertices[0][2]
        for v in self.vertices:
            if v[0]<self.mmx[0]:
                self.mmx[0]=v[0]
            elif v[0]>self.mmx[1]:
                self.mmx[1]=v[0]
            if v[1]<self.mmy[0]:
                self.mmy[0]=v[1]
            elif v[1]>self.mmy[1]:
                self.mmy[1]=v[1]
            if v[2]<self.mmz[0]:
                self.mmz[0]=v[2]
            elif v[2]>self.mmz[1]:
                self.mmz[1]=v[2]

	return self.mmx,self.mmy,self.mmz

    def seperate(self):
	ver=list()
	v1=list()
	v2=list()
	for v in self.vertices:
		if(v[0]>0):
			ver.append(v)
	ver.sort(key = sortFirst)
	while True:
		for i in range(0,len(ver)):
			if(i<len(ver)/2):
				v1.append(ver[i])
			else:
				v2.append(ver[i])
		l1,mmz1=Length(v1)
		l2,mmz2=Length(v2)
		if(l1<l2):
			if(mmz1>self.mz):
				self.mz=mmz1
			ver=v2
		else:
			break;
	shiftx=(mmx[1]-mmx[0])/2-mmx[1]
	shifty=(mmy[1]-mmy[0])/2-mmy[1]
	shiftz=(mmz[1]-mmz[0])/2-mmz[1]
	for i in range(0,len(self.vertices)):
		self.vertices[i][0]+=shiftx
		self.vertices[i][1]+=shifty
		self.vertices[i][2]+=shiftz

    def render_scene(self):
        global x
        global y
        global z
        global ax
        global ay
        global az
        if len(self.faces) > 0:
	    glPushMatrix()
	    glShadeModel(GL_SMOOTH)
	    glLoadIdentity()
	    glMatrixMode(GL_MODELVIEW)
            light0_diffuse = (1.0, 1.0, 1.0, 1.0)
            light0_position = (1.0, 1.0, 1.0, 0.0)
            glLightfv(GL_LIGHT0, GL_POSITION, light0_position)
            glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse)
            glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0)
            glPolygonMode(GL_FRONT, GL_FILL)
	    
            glTranslatef(x, y, z)
            glRotate(ax,0,1,0)
            glRotate(ay,0,0,1)
            glRotate(az,1,0,0)	    
            for face in self.faces:
                glBegin(GL_POLYGON)
                for v in face:
                    vertexDraw = self.vertices[int(v[0]) - 1]
		    if vertexDraw[2]>=self.mz or (vertexDraw[2]<=self.mz and vertexDraw[0]>0 and ax<0) or (vertexDraw[2]<=self.mz and vertexDraw[0]<0 and ax>0):
                    	glNormal3f(self.normals[int(v[2])-1][0],self.normals[int(v[2])-1][1],self.normals[int(v[2])-1][2])
                    	glVertex3fv(vertexDraw)
                glEnd()
	    glPopMatrix()
            glDisable(GL_LIGHTING)
            glDisable(GL_LIGHT0)
            ##


class objItem(object):

    def __init__(self):
        self.vertices = []
        self.faces = []
	global x
	global y
        global z
	global ax
	global ay
        global az
        ax = 0
        ay = 0
        az = 0
        x = 0
        y = 0
        z = 0
        self.obj = ObjLoader()    
    def loadobj(self):
        self.obj.newload(glass[currentobj])
        
    def render_scene(self):
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
    def setplace(self,a,b,c):
	global x
	global y
        global z
        x=a
        y=b
        z=c
        
    def rotatex(self, n):
        global ax
        ax=n

    def rotatey(self, n):
        global ay
        ay=n
    def rotatez(self, n):
        global az
        az=n        
        ##


object = objItem()
object.loadobj()
global mmx,mmy,mmz
mmx,mmy,mmz = object.obj.minmax()
global lenx
lenx=mmx[1]-mmx[0]
object.obj.seperate()

def cv2array(im): 
    h,w,c=im.shape
    a = np.fromstring( 
       im.tostring(), 
       dtype=im.dtype, 
       count=w*h*c) 
    a.shape = (h,w,c) 
    return a

def init():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glutDisplayFunc(display)
    glutReshapeFunc(reshape)
    glutKeyboardFunc(keyboard)
    glutIdleFunc(idle)
    
def idle():
    global capture
    ret,image = capture.read()
    image=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    glEnable(GL_TEXTURE_2D)
    texid=glGenTextures(1) 
    glBindTexture(GL_TEXTURE_2D, texid)
    borderColor = ( 1.0, 1.0, 0.0, 1.0 )
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,GL_REPEAT) 
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    

 
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640 , width+height-640, 0,GL_RGB, GL_UNSIGNED_BYTE,image)	
    glutPostRedisplay()

def display():
    glEnable(GL_TEXTURE_2D)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluOrtho2D(0, 635, 0, 480)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    
    glPushMatrix()
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 0.0)
    glVertex2f(0.0, 480.0)
    glTexCoord2f(0.0, 1.0)
    glVertex2f(0.0, 0.0)
    glTexCoord2f(1.0, 1.0)
    glVertex2f(635.0, 0.0)
    glTexCoord2f(1.0, 0.0)
    glVertex2f(635, 480.0)
    glEnd()
    glPopMatrix()
    global ret
    ret,img = capture.read()
    global co
    global detector
    global predictor
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glEnable(GL_CULL_FACE)
    glClear(GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(30,635/480, 1,1000)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = detector(gray)
    if len(faces)==0:
        co+=1
        print("No faces Detected",co)
    elif len(faces)>10:
        co+=1
        print("more than 1 face Detected",co)
    else:
        for face in faces:
            x1 = face.left()
            y1 = face.top()
            x2 = face.right()
            y2 = face.bottom()

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            landmarks = predictor(gray, face)

            x27=landmarks.part(27).x
            y27 = landmarks.part(27).y
	    x28=landmarks.part(28).x
            y28 = landmarks.part(28).y
            x8=landmarks.part(8).x
            y8 = landmarks.part(8).y
            x0=landmarks.part(0).x
            y0 = landmarks.part(0).y
            x16=landmarks.part(16).x
            y16 = landmarks.part(16).y
            x33=landmarks.part(33).x
            y33=landmarks.part(33).y
            d1=math.sqrt((x27-x0)*(x27-x0)+(y27-y0)*(y27-y0))
            d2=math.sqrt((x27-x16)*(x27-x16)+(y27-y16)*(y27-y16))
            d3=math.sqrt((x33-x27)*(x33-x27)+(y27-y33)*(y27-y33))
            d4=math.sqrt((x8-x33)*(x8-x33)+(y8-y33)*(y8-y33))
	    global lenx
            dist=-lenx*1200/(d1+d2)
            if d1>d2 and d1!=0:
                object.rotatex((90-math.tan(d2/d1)*180/3.14)/4)
            elif d2!=0 and d2>d1:
                object.rotatex((-90+math.tan(d1/d2)*180/3.14)/4)
            if d3>d4 and d3!=0:
                object.rotatez((90-math.tan(d4/d3)*180/3.14)/6)
            elif d4!=0 and d4>d3:
                object.rotatez((-90+math.tan(d3/d4)*180/3.14)/6)
            if x8>x27:
                object.rotatey((90-math.atan((y8-y27)/(x8-x27))*180/3.14))
            elif x8<x27:
                object.rotatey((-90-math.atan((y8-y27)/(x8-x27))*180/3.14))
	    
	    global ax
	    if ax>1:
		x27-=15
	    if ax<-1:
		x27+=15
	    global ay
	    if(ay>0):
		y27+=22
	    elif(ay<0):
		y27+=22
            xx=-math.tan((x27-635/2)*30*3.14/(635/2*400))*dist
	    yy=math.tan((y27-480/2)*30*3.14/(480/2*400))*dist

            object.setplace( xx, yy, dist)
            object.render_scene()
            object.obj.render_scene()

    glFlush()
    glutSwapBuffers()
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

def reshape(w, h):
    if h == 0:
      h = 1

    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)

    glLoadIdentity()
    
    if w <= h:
      glOrtho(-nRange, nRange, -nRange*h/w, nRange*h/w, -nRange, nRange)
    else:
      glOrtho(-nRange*w/h, nRange*w/h, -nRange, nRange, -nRange, nRange)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def keyboard(key, x, y):
    global anim
    global currentobj
    global lenx
    global mmx,mmy,mmz
    if key == chr(27):
	cap.release()
	cv2.destroyAllWindows()
	sys.exit()
    if key == 'p' and currentobj!=0:
        print("key 1 press ")
        currentobj-=1
        object.loadobj()
        mmx,mmy,mmz = object.obj.minmax()
	lenx=mmx[1]-mmx[0]
	object.obj.seperate()
    if key =='n' and currentobj<len(glass)-1:
        print("key2 press")
        currentobj+=1
        object.loadobj()
        mmx,mmy,mmz = object.obj.minmax()
        lenx=mmx[1]-mmx[0]
	object.obj.seperate()


def main():
	global capture
	capture = cv2.VideoCapture(0)

	capture.set(10,width)
	capture.set(4,height)
	glutInit()
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
	glutInitWindowSize(width, height)
	glutInitWindowPosition(100, 100)
	glutCreateWindow("OpenGL + OpenCV")
	init()
	glutMainLoop()

main()
