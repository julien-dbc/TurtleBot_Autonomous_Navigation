# 🐢 TurtleBot Autonomous Navigation

<div align="center">
  <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python" />
  <img src="https://img.shields.io/badge/ROS-22304E?style=for-the-badge&logo=ros&logoColor=white" alt="ROS" />
  <img src="https://img.shields.io/badge/Gazebo-FF8C00?style=for-the-badge&logo=gazebo&logoColor=white" alt="Gazebo" />
  <img src="https://img.shields.io/badge/Robotics-4CAF50?style=for-the-badge&logo=robot&logoColor=white" alt="Robotics" />
</div>

---

## 🇬🇧 English Version

### 📖 Overview

* 🎯 **Challenge:** Enable a TurtleBot to navigate autonomously in an unknown environment while safely avoiding obstacles.
* 🛠️ **Solution:** Developed a navigation stack using ROS and Python within a Gazebo simulation. Implemented a robust State Machine processing Lidar data for dynamic path planning and managing physical collisions via bumper sensors.
* 📈 **Impact:** Achieved reliable autonomous movement and collision avoidance, demonstrating strong proficiency in standard robotics frameworks (ROS/Gazebo).

### 🧠 System Architecture

The core of this project relies on a Finite State Machine (FSM) that dictates the robot's behavior based on sensor inputs.

1. **Collision Recovery (Bumper):** The robot reacts to physical contact detected by its bumper. Upon collision, it executes a strict sequence: it stops immediately, reverses to clear the obstacle, rotates approximately 90 degrees, and then resumes its autonomous forward movement.
2. **Anticipatory Avoidance (Lidar):** To prevent physical contacts, the robot uses a simulated Lidar (via a Kinect depth camera). It scans a field of view of ±80 degrees ahead of the robot. 
3. **Dynamic Speed & Steering:** As the robot approaches an obstacle, it proportionally reduces its speed. It also calculates obstacle density on its left and right sides to steer towards the clearest path.
4. **Safety Priority:** In the FSM logic, physical collisions (Bumper) always take priority over Lidar detections to ensure maximum safety if the avoidance algorithm fails.

### 💻 Code Usage

The main logic is contained within the `seatech_sm.py` script. 
This script acts as a ROS node (`joy4ctrl`) that subscribes to sensor topics (joystick, bumper events, and laser scans) and publishes velocity commands to the `mobile_base/commands/velocity` topic. 

To use this project globally:
1. Ensure you have a working ROS environment with Gazebo and the TurtleBot2 packages installed.
2. Run the simulation environment.
3. Execute the `seatech_sm.py` script. The FSM will initialize, and you can use a connected joystick to toggle between manual control and the autonomous navigation mode.

### 📂 Documentation & Media

* **Video Demonstration:** See `turtle.mp4` for a demonstration of the robot navigating in both simulated and real environments.
* **Images:** Check out `IMG_2701.jpg` for a look at the physical TurtleBot hardware.
* **Full Report:** A detailed project report (`TurtleBOT.pdf`) is included in this repository. It covers the complete FSM state diagrams, detailed logic explanations, and simulation vs. reality constraints.

### ⚖️ Copyright & License

The code, media, and report included in this repository are part of an engineering school project (SeaTech). The academic report and its contents are subject to copyright. You are free to explore the code for educational purposes, but reproduction or direct academic reuse of the report without proper citation is prohibited.

---
---

## 🇫🇷 Version Française

### 📖 Vue d'ensemble

* 🎯 **Défi :** Permettre à un TurtleBot de naviguer de manière autonome dans un environnement inconnu tout en évitant les obstacles en toute sécurité.
* 🛠️ **Solution :** Développement d'une stack de navigation utilisant ROS et Python dans une simulation Gazebo. Implémentation d'une machine à états robuste traitant les données Lidar pour la planification dynamique de trajectoire et gérant les collisions physiques via les capteurs du pare-chocs (bumper).
* 📈 **Impact :** Obtention d'un mouvement autonome fiable et d'un évitement de collisions fluide, démontrant une forte maîtrise des frameworks de robotique standards (ROS/Gazebo).

### 🧠 Architecture du Système

Le cœur de ce projet repose sur une Machine à États Finis (FSM) qui dicte le comportement du robot en fonction des entrées des capteurs.

1. **Récupération après collision (Bumper) :** Le robot réagit au contact physique. Lors d'une collision, il exécute une séquence stricte : arrêt immédiat, recul pour se dégager, rotation d'environ 90 degrés, puis reprise du mouvement autonome.
2. **Évitement anticipé (Lidar) :** Pour prévenir les contacts physiques, le robot utilise un Lidar simulé (caméra de profondeur Kinect). Il scanne un champ de vision de ±80 degrés devant lui.
3. **Vitesse et direction dynamiques :** En s'approchant d'un obstacle, le robot réduit proportionnellement sa vitesse. Il calcule également la densité des obstacles à gauche et à droite pour se diriger vers le chemin le plus dégagé.
4. **Priorité à la sécurité :** Dans la logique FSM, les chocs physiques (Bumper) sont toujours prioritaires sur les détections Lidar pour assurer une sécurité maximale si l'algorithme d'évitement échoue.

### 💻 Utilisation du Code

La logique principale est contenue dans le script `seatech_sm.py`.
Ce script agit comme un nœud ROS (`joy4ctrl`) qui s'abonne aux topics des capteurs (joystick, bumper, scan laser) et publie des commandes de vitesse sur le topic `mobile_base/commands/velocity`.

Pour utiliser ce projet :
1. S'assurer d'avoir un environnement ROS fonctionnel avec Gazebo et les paquets TurtleBot2 installés.
2. Lancer l'environnement de simulation.
3. Exécuter le script `seatech_sm.py`. La machine à états s'initialisera, et vous pourrez utiliser un joystick connecté pour basculer entre le contrôle manuel et le mode de navigation autonome.

### 📂 Documentation & Médias

* **Démonstration vidéo :** Voir `turtle.mp4` pour une démonstration du robot naviguant dans des environnements simulés et réels.
* **Images :** Voir `IMG_2701.jpg` pour un aperçu du matériel physique TurtleBot.
* **Rapport complet :** Un rapport détaillé du projet (`TurtleBOT.pdf`) est inclus dans ce dépôt. Il couvre les diagrammes d'états complets de la FSM, les explications détaillées de la logique et les contraintes entre la simulation et la réalité.

### ⚖️ Droits d'auteur & Licence

Le code, les médias et le rapport inclus dans ce dépôt font partie d'un projet d'école d'ingénieurs (SeaTech). Le rapport académique et son contenu sont soumis aux droits d'auteur. Vous êtes libre d'explorer le code à des fins éducatives, mais toute reproduction ou réutilisation académique directe du rapport sans citation appropriée est interdite.
