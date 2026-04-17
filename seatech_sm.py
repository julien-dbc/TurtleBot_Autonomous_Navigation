#!/usr/bin/env python2
# -*- coding: utf-8 -*-
""" ROS python pogramming with finite state machines to describe a robot's behaviors
    Vincent Hugel
    Seatech/SYSMER 2A Course
    free to use so long as the author and other contributers are credited.
"""
#############################################################################
# imports
#############################################################################
import rospy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from fsm import fsm
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan  # type de message pour le lidar
from nav_msgs.msg import Odometry # l'odometrie du robot
import tf # importation de fonctions utiles de transformations de repere (tf : transformation de repere)
from tf.transformations import *


#############################################################################
# class RobotBehavior
#############################################################################
class RobotBehavior(object):
	#############################################################################
	# constructor, called at creation of instance
	#############################################################################
	def __init__(self, handle_pub, T):
		self.twist = Twist()
		self.twist_real = Twist()
		self.vreal = 0.0 # longitudial velocity
		self.wreal = 0.0 # angular velocity
		self.vmax = 1.5 # Vitesse lineaire max
		self.wmax = 4.0 # Vitesse angulaire max
		
		# Gestion du Joystick
		self.previous_signal = 0
		self.button_pressed = False
		self.joy_activated = False
		
		self.pub = handle_pub
		self.T = T
		
		#Bumper
		self.Obsdetect=False;
		
		# Lidar
		self.Lidardetect=False;
		
		#Compteurs
		self.cpt1=0
		self.cpt2=0
		self.cpt3=0
		self.cpt4=0
		self.cpt5=0
		
		# Variables pour la logique d'évitement Lidar
		self.avoid_direction = 0 # 0=Aucun, 1=Gauche, -1=Droite
		self.min_dist_lidar = 99.0 # Distance minimale détectée
		
		# --- Définition de la Machine à États Finis (FSM) ---
        	# Structure : (Source, Destination, Condition de transition, Action/Callback)
		self.fs = fsm([ # Démarrage vers contrôle manette
			    ("Start","JoyControl", True ),
			    
			    # Transitions Manette <-> Autonome
			    ("JoyControl","AutonomousMode1", self.check_JoyControl_To_AutonomousMode1, self.DoAutonomousMode1),
			    ("JoyControl","JoyControl", self.KeepJoyControl, self.DoJoyControl),
			    ("AutonomousMode1","JoyControl", self.check_AutonomousMode1_To_JoyControl, self.DoJoyControl),
			    ("AutonomousMode1","AutonomousMode1", self.KeepAutonomousMode1, self.DoAutonomousMode1),
			    
			    # Gestion collision Bumper (Prioritaire)
			    ("AutonomousMode1","Stop1",self.check_AutonomousMode1_To_Stop1,self.DoStop1),
			    ("Stop1","Stop1",self.KeepStop1,self.DoStop1),
			    
			    # Gestion évitement Lidar
			    ("AutonomousMode1","AvoidObstacle",self.check_AutonomousMode1_To_AvoidObstacle,self.DoAvoidObstacle),
			    ("AvoidObstacle","AvoidObstacle",self.KeepAvoidObstacle,self.DoAvoidObstacle),
			    ("AvoidObstacle","AutonomousMode1",self.check_AvoidObstacle_To_AutonomousMode1,self.DoAutonomousMode1),
			    
			    # Séquence de dégagement après Bumper (Stop -> Recule -> Stop -> Tourne -> Stop -> Repart)
			    ("Stop1","Recule",self.check_Stop1_To_Recule,self.DoRecule),
			    ("Recule","Recule",self.KeepRecule,self.DoRecule),
			    ("Recule","Stop2",self.check_Recule_To_Stop2,self.DoStop2),
			    ("Stop2","Stop2",self.KeepStop2,self.DoStop2),
			    ("Stop2","Rotate90",self.check_Stop2_To_Rotate90,self.DoRotate90),
			    ("Rotate90","Rotate90",self.KeepRotate90,self.DoRotate90),
			    ("Rotate90","Stop3",self.check_Rotate90_To_Stop3,self.DoStop3),
			    ("Stop3","Stop3",self.KeepStop3,self.DoStop3),
			    ("Stop3","AutonomousMode1",self.check_Stop3_To_AutonomousMode1,self.DoAutonomousMode1)
			    ] )
		


	#############################################################################
	# callback for joystick feedback
	#############################################################################
	def callback(self,data):
	    	# Mapping des axes manette vers vitesses linéaire et angulaire
		self.twist.linear.x = self.vmax * data.axes[1]
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = self.wmax*data.axes[3]
		
	
		# Gestion du bouton pour changer d'état (Manette <-> Autonome)
		if (not self.button_pressed):
			self.button_pressed = (self.previous_signal==0 and data.buttons[0]==1)		
		self.previous_signal = data.buttons[0];
		
		# Détection d'activité sur le joystick (pour reprendre la main)
		self.joy_activated = (abs(data.axes[1])>0.001 or abs(data.axes[3])>0.001)


	#############################################################################
	# smoothing velocity function to avoid brutal change of velocity
	#############################################################################
	def smooth_velocity(self):
		accmax = 0.01;
		accwmax = 0.05;
		vjoy = 0.0
		wjoy = 0.0
		vold = 0.0
		wold = 0.0	

		#filter twist
		vjoy = self.twist.linear.x
		vold = self.vreal
		deltav_max = accmax / self.T
	
		#vreal
		if abs(vjoy - self.vreal) < deltav_max:
			self.vreal = vjoy
		else:
			sign_ = 1.0
			if (vjoy < self.vreal):
				sign_ = -1.0
			else:
				sign_ = 1.0
			self.vreal = vold + sign_ * deltav_max
	
		#saturation
		if (self.vreal > self.vmax):
			self.vreal = self.vmax
		elif (self.vreal < -self.vmax):
			self.vreal = -self.vmax		
	
		#filter twist
		wjoy = self.twist.angular.z
		wold = self.wreal
		deltaw_max = accwmax / self.T
	
		#wreal
		if abs(wjoy - self.wreal) < deltaw_max:
			self.wreal = wjoy
		else:
			sign_ = 1.0
			if (wjoy < self.wreal):
				sign_ = -1.0
			else:
				sign_ = 1.0
			self.wreal = wold + sign_ * deltaw_max
		#saturation
		if (self.wreal > self.wmax):
			self.wreal = self.wmax
		elif (self.wreal < -self.wmax):
			self.wreal = -self.wmax		
			
		self.twist_real.linear.x = self.vreal	
		self.twist_real.angular.z = self.wreal
	
	#############################################################################
	# Conditions de transitions (CHECK functions)
	#############################################################################
	# --- Transitions Manette / Autonome ---
	def check_JoyControl_To_AutonomousMode1(self,fss):
		return self.button_pressed

	def check_AutonomousMode1_To_JoyControl(self,fss):
		return self.joy_activated
	
	# --- Transitions liées aux Capteurs ---	
	def check_AutonomousMode1_To_Stop1(self,fss):
		print(self.Obsdetect)
		if(self.Obsdetect==True): # Priorité au bumper (choc physique)
			return True
		else:
			return False
	#lidar	####################################################	
	def check_AutonomousMode1_To_AvoidObstacle(self,fss):
		return self.Lidardetect # Détection lidar (sans contact)
	

	
	def check_AvoidObstacle_To_AutonomousMode1(self,fss):
		if not self.Lidardetect:
			self.avoid_direction = 0  # RESET direction d’évitement
			return True
		return False
			
	########################################################""
	# --- Transitions temporisées (Séquence de dégagement) ---
    	# On utilise des compteurs (cpt) qui s'incrémentent à chaque boucle (Hz)
    	# Seuil = 10 cycles (donc 1 seconde si Hz=10)
    
	def check_Stop1_To_Recule(self,fss):
		seuil=10
		if(self.cpt1)>seuil:
			return True
		else:
			return False
	
	def check_Recule_To_Stop2(self,fss):
		seuil=10
		if(self.cpt2)>seuil:
			return True
		else:
			return False
	
	def check_Stop2_To_Rotate90(self,fss):
		seuil=10
		if(self.cpt3)>seuil:
			return True
		else:
			return False
	
	def check_Rotate90_To_Stop3(self,fss):
		seuil=10
		if(self.cpt4)>seuil:
			return True
		else:
			return False
	def check_Stop3_To_AutonomousMode1(self,fss):
		seuil=10
		if(self.cpt5)>seuil:
			return True
		else:
			return False
	
	# --- Conditions de maintien d'état (KEEP functions) ---

	def KeepJoyControl(self,fss):
		return (not self.check_JoyControl_To_AutonomousMode1(fss))
	def KeepAutonomousMode1(self, fss):
    	# Retourne True seulement si AUCUNE transition n'est activée
		joy = self.check_AutonomousMode1_To_JoyControl(fss)
		stop1 = self.check_AutonomousMode1_To_Stop1(fss)
		avoid = self.check_AutonomousMode1_To_AvoidObstacle(fss)	    			
		return not (joy or stop1 or avoid)		
	def KeepStop1(self,fss):
		return (not self.check_Stop1_To_Recule(fss))
	def KeepRecule(self,fss):
		return (not self.check_Recule_To_Stop2(fss))
	def KeepStop2(self,fss):
		return (not self.check_Stop2_To_Rotate90(fss))
	def KeepRotate90(self,fss):
		return (not self.check_Rotate90_To_Stop3(fss))
	def KeepStop3(self,fss):
		return (not self.check_Stop3_To_AutonomousMode1(fss))
	def KeepAvoidObstacle(self,fss):
		return (not self.check_AvoidObstacle_To_AutonomousMode1(fss))

	#############################################################################
	# Actions des états (DO functions)
	#############################################################################
	def DoJoyControl(self,fss,value):
		self.button_pressed =  False;
		self.smooth_velocity()
		self.pub.publish(self.twist_real)

	def DoAutonomousMode1(self,fss,value):
		print("AutonomousMode1")
		# Réinitialisation des variables d'état
		self.avoid_direction = 0
		self.Obsdetect=False
		self.cpt5=0
		self.button_pressed =  False;
		# go forward
		go_fwd = Twist()
		MAX_AUTONOMOUS_SPEED = self.vmax / 1.5  # Vitesse max (ex: 0.5 m/s)
        	SAFE_DISTANCE = 2.0 # Distance (m) à laquelle on atteint la vitesse max
        
        # Adaptation de la vitesse : Plus on est proche, plus on ralentit (P-Control simple)
        	speed_factor = self.min_dist_lidar / SAFE_DISTANCE
        
        # Saturation (pour ne pas s'arrêter si l'objet est loin, ni aller trop vite)
        	if speed_factor < 0.2:
            		speed_factor = 0.1 # Vitesse minimale pour avancer
        	if speed_factor > 1.0:
            		speed_factor = 1.0
            
        	go_fwd.linear.x = MAX_AUTONOMOUS_SPEED * speed_factor
        
        # Si un obstacle est très proche (mais pas assez pour changer d'état), 
        # commencer à tourner préventivement.
        	if self.min_dist_lidar < (SAFE_DISTANCE / 2.0):
             		go_fwd.angular.z = 0.3 # Tourner doucement
        	else:
             		go_fwd.angular.z = 0.0

        	rospy.loginfo("Mode Autonome: Vitesse = {:.2f} (Dist min: {:.2f}m)".format(go_fwd.linear.x, self.min_dist_lidar))
        	self.pub.publish(go_fwd)
	
	# --- Séquence Bumper (Stop -> Recule -> Stop -> Rotate -> Stop) ---	
	def DoStop1(self,fss,value):
		print("Stop1")
		self.cpt1+=1
		go_fwd = Twist()
		go_fwd.linear.x = 0
		self.pub.publish(go_fwd) # Envoi vitesse 0
	
	def DoRecule(self,fss,value):
		print("Recule")
		self.cpt1=0
		self.cpt2+=1
		go_fwd = Twist()
		go_fwd.linear.x = -self.vmax/3.0 # Recul lent
		self.pub.publish(go_fwd)
	
	def DoStop2(self,fss,value):
		print("Stop2")
		self.cpt2=0
		self.cpt3+=1

		go_fwd = Twist()
		go_fwd.linear.x = 0
		self.pub.publish(go_fwd)
	
	def DoRotate90(self,fss,value):
		print("Rotate90")
		self.cpt3=0
		self.cpt4+=1
		go_fwd = Twist()
		go_fwd.angular.z=self.wmax/2.0 # Rotation sur place
		self.pub.publish(go_fwd)
		
	def DoStop3(self,fss,value):
		print("Stop3")
		self.cpt4=0
		self.cpt5+=1
		go_fwd = Twist()
		go_fwd.angular.z = 0
		self.pub.publish(go_fwd)
		
	# --- Logique d'évitement Lidar ---
		
	def DoAvoidObstacle(self, fss, value):
        	print("AvoidObstacle - Évitement lidar actif")
    		if not self.Lidardetect:
			self.avoid_direction = 0
        	avoid_cmd = Twist()
        
        # Sécurité : si plus de données, on s'arrête
        	if not (hasattr(self, 'obstacles') and self.obstacles):
            # Cas de repli : l'état a été conservé mais les obstacles ont disparu.
			self.avoid_direction = 0
            		avoid_cmd.linear.x = 0.0
           		avoid_cmd.angular.z = 0.0
            		self.pub.publish(avoid_cmd)
            		return

        	distances = [dist for angle, dist in self.obstacles]
        	distance_min = min(distances) if distances else 1.0
        
        # --- NOUVELLE LOGIQUE DE RÉACTION ---
        	# 1. Prise de décision de la direction (Gauche ou Droite ?)
        	if self.avoid_direction == 0:
        		# On compte les obstacles à gauche (>0) et à droite (<=0)
            		left_obstacles = sum(1 for angle, dist in self.obstacles if angle > 0)
            		right_obstacles = sum(1 for angle, dist in self.obstacles if angle <= 0)
            		
            		# On va là où il y a le moins d'obstacles
            		if left_obstacles > right_obstacles:
                		self.avoid_direction = -1 # Tourner à DROITE (moins d'obstacles)
                		rospy.loginfo("Obstacle détecté, DÉCISION: tourner à DROITE")
            		else:
                		self.avoid_direction = 1  # Tourner à GAUCHE
               			rospy.loginfo("Obstacle détecté, DÉCISION: tourner à GAUCHE")

        # 2. Appliquer la stratégie d'évitement en fonction de la distance
        #    MAIS en gardant la direction mémorisée
        
        	if distance_min < 0.35:  # Zone d'Arrêt d'urgence
            		avoid_cmd.linear.x = 0.0 # Arrêt complet
        	elif distance_min < 0.6: # Zone de manœuvre
            		avoid_cmd.linear.x = 0.1 # Avancer très lentement
        	else: # Zone de pré-alerte
            		avoid_cmd.linear.x = 0.15 

        # 3. Appliquer la rotation mémorisée
        	if self.avoid_direction == 1:
            		avoid_cmd.angular.z = 1.0  # GAUCHE
        	else:
            		avoid_cmd.angular.z = -1.0 # DROITE

        # Cas d'URGENCE (trop proche) : annule tout et recule
        	if distance_min < 0.20:
            		rospy.loginfo("URGENCE CRITIQUE: RECUL!")
            		avoid_cmd.linear.x = -0.3   # Recul rapide
            # Tourne dans la direction opposée à la direction mémorisée pour s'éloigner
            		avoid_cmd.angular.z = -0.5 * self.avoid_direction 
            
        	self.pub.publish(avoid_cmd)
	
	
	def processBump(self,data):
		# Si on n'est pas déjà en collision, on active le flag au moindre contact
		if (self.Obsdetect==False):
			if(data.state==BumperEvent.PRESSED):
				self.Obsdetect=True
#############################################################################
#############################################################################
	def processScan(self, data):
        	detected_now = False
        	dist_detect = 1.0  # Seuil de détection (mètres)
        	# Augmenter le FOV pour détecter les obstacles sur les côtés
        	fov_deg = 80       # Champ de vision surveillé (+/- 80 degrés)
        	fov_rad = math.radians(fov_deg)
        	count=0
        	obstacles = []
        	
        	current_min_dist = 99.0

        	for i, value in enumerate(data.ranges):
            	# Assurez-vous que la distance est valide et dans les limites du capteur
            		if math.isnan(value) or math.isinf(value) or value < data.range_min:
                		continue
            		# Calcul de l'angle du point courant
            		angle = data.angle_min + i * data.angle_increment
            		# Si le point est dans le champ de vision (FOV)
            		if -fov_rad <= angle <= fov_rad:
                		if value < current_min_dist:
                    			current_min_dist = value
                
                		# Si l'obstacle est proche
                		if value < dist_detect:
                    			count+=1
                    			# Filtre de bruit : il faut au moins 3 points consécutifs pour valider un obstacle
                    			if count >= 3: 
                        			detected_now = True #self.Lidardetect = True
                        			obstacles.append((angle, value))
        	self.obstacles = obstacles
		self.min_dist_lidar = current_min_dist
		self.Lidardetect = detected_now
		
        	if self.Lidardetect:
            		rospy.loginfo("Lidar: Obstacle detecte (d < {}m)".format(dist_detect))
			

#############################################################################
# main function
#############################################################################
if __name__ == '__main__':
	try:
		rospy.init_node('joy4ctrl')
		# Topic pour Turtlebot 2
		pub = rospy.Publisher('mobile_base/commands/velocity', Twist,queue_size=10)
		Hz = 10
		rate = rospy.Rate(Hz)
		T = 1.0/Hz
		
		MyRobot = RobotBehavior(pub,T);
		
		# Abonnement aux capteurs
		rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,MyRobot.processBump)
		rospy.Subscriber("joy", Joy, MyRobot.callback)
		rospy.Subscriber("scan", LaserScan, MyRobot.processScan) 
		
		# Démarrage de la machine à états
		MyRobot.fs.start("Start")

		# loop at rate Hz
		while (not rospy.is_shutdown()): # Boucle principale : mise à jour de la FSM
			ret = MyRobot.fs.event("")
			rate.sleep()

	except rospy.ROSInterruptException:
        	pass
