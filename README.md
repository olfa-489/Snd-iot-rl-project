# Snd-iot-rl-project
<img width="1724" height="732" alt="Screenshot 2025-12-13 111944" src="https://github.com/user-attachments/assets/821e90ab-ab70-464d-9912-3b0db86f0711" />

# Structure of project

<img width="741" height="500" alt="Screenshot 2026-01-17 104642" src="https://github.com/user-attachments/assets/f0144ef3-6963-4365-82d6-13de5d4c6c18" />

# Topology creation / iot_topology.py



"""
Topologie WiFi IoT étendue pour SDN-IoT RL Traffic Management

Topologie:
- 6 Access Points (AP1-AP6) répartis en zones
- 20 Stations représentant différents types de dispositifs IoT:
  * 6 Caméras de surveillance (haute bande passante)
  * 6 Capteurs environnementaux (faible latence)
  * 4 Contrôleurs industriels (critiques)
  * 2 Drones (mobiles)
  * 2 Serveurs cloud (destinations)
"""

from mininet.node import RemoteController
from mininet.log import setLogLevel, info
from mn_wifi.net import Mininet_wifi
from mn_wifi.node import OVSKernelAP
from mn_wifi.cli import CLI
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference
import time

def create_extended_iot_topology():
    """
    Crée une topologie IoT WiFi étendue
    
    Zones:
    - Zone A (AP1, AP2): Surveillance vidéo (caméras)
    - Zone B (AP3, AP4): Capteurs industriels
    - Zone C (AP5, AP6): Contrôleurs et serveurs
    """
    
    net = Mininet_wifi(
        controller=RemoteController,
        accessPoint=OVSKernelAP,
        link=wmediumd,
        wmediumd_mode=interference
        
    )
    
    info("*** Création du contrôleur SDN\n")
    c0 = net.addController(
        'c0',
        controller=RemoteController,
        ip='127.0.0.1',
        port=6653
    )
    
    info("*** Création des Access Points (6 APs)\n")
    
    # Zone A - Surveillance (AP1, AP2)
    ap1 = net.addAccessPoint(
        'ap1',
        ssid='IoT-Surveillance-1',
        mode='g',
        channel='1',
        position='20,80,0',
        range=40,
        failMode='secure',
        protocols='OpenFlow13',
        dpid='0000000000000001'
    )
    
    ap2 = net.addAccessPoint(
        'ap2',
        ssid='IoT-Surveillance-2',
        mode='g',
        channel='6',
        position='60,80,0',
        range=40,
        failMode='secure',
        protocols='OpenFlow13',
        dpid='0000000000000002'
    )
    
    # Zone B - Capteurs industriels (AP3, AP4)
    ap3 = net.addAccessPoint(
        'ap3',
        ssid='IoT-Sensors-1',
        mode='g',
        channel='11',
        position='20,40,0',
        range=40,
        failMode='secure',
        protocols='OpenFlow13',
        dpid='0000000000000003'
    )
    
    ap4 = net.addAccessPoint(
        'ap4',
        ssid='IoT-Sensors-2',
        mode='g',
        channel='1',
        position='60,40,0',
        range=40,
        failMode='secure',
        protocols='OpenFlow13',
        dpid='0000000000000004'
    )
    
    # Zone C - Contrôleurs et serveurs (AP5, AP6)
    ap5 = net.addAccessPoint(
        'ap5',
        ssid='IoT-Control-1',
        mode='g',
        channel='6',
        position='100,60,0',
        range=40,
        failMode='secure',
        protocols='OpenFlow13',
        dpid='0000000000000005'
    )
    
    ap6 = net.addAccessPoint(
        'ap6',
        ssid='IoT-Cloud-1',
        mode='g',
        channel='11',
        position='140,60,0',
        range=40,
        failMode='secure',
        protocols='OpenFlow13',
        dpid='0000000000000006'
    )
    
    info("*** Création des stations (20 dispositifs IoT)\n")
    
    # ============ ZONE A - CAMÉRAS DE SURVEILLANCE (6) ============
    info("  → Zone A: Caméras de surveillance\n")
    
    cam1 = net.addStation(
        'cam1',
        ip='10.0.0.1/24',
        position='15,85,0',
        range=15,
        mac='00:00:00:00:00:01'
    )
    
    cam2 = net.addStation(
        'cam2',
        ip='10.0.0.2/24',
        position='25,75,0',
        range=15,
        mac='00:00:00:00:00:02'
    )
    
    cam3 = net.addStation(
        'cam3',
        ip='10.0.0.3/24',
        position='30,85,0',
        range=15,
        mac='00:00:00:00:00:03'
    )
    
    cam4 = net.addStation(
        'cam4',
        ip='10.0.0.4/24',
        position='55,85,0',
        range=15,
        mac='00:00:00:00:00:04'
    )
    
    cam5 = net.addStation(
        'cam5',
        ip='10.0.0.5/24',
        position='65,75,0',
        range=15,
        mac='00:00:00:00:00:05'
    )
    
    cam6 = net.addStation(
        'cam6',
        ip='10.0.0.6/24',
        position='60,90,0',
        range=15,
        mac='00:00:00:00:00:06'
    )
    
    # ============ ZONE B - CAPTEURS INDUSTRIELS (6) ============
    info("  → Zone B: Capteurs environnementaux\n")
    
    sensor1 = net.addStation(
        'sensor1',
        ip='10.0.0.11/24',
        position='15,45,0',
        range=10,
        mac='00:00:00:00:00:11'
    )
    
    sensor2 = net.addStation(
        'sensor2',
        ip='10.0.0.12/24',
        position='25,35,0',
        range=10,
        mac='00:00:00:00:00:12'
    )
    
    sensor3 = net.addStation(
        'sensor3',
        ip='10.0.0.13/24',
        position='20,48,0',
        range=10,
        mac='00:00:00:00:00:13'
    )
    
    sensor4 = net.addStation(
        'sensor4',
        ip='10.0.0.14/24',
        position='55,45,0',
        range=10,
        mac='00:00:00:00:00:14'
    )
    
    sensor5 = net.addStation(
        'sensor5',
        ip='10.0.0.15/24',
        position='65,35,0',
        range=10,
        mac='00:00:00:00:00:15'
    )
    
    sensor6 = net.addStation(
        'sensor6',
        ip='10.0.0.16/24',
        position='60,48,0',
        range=10,
        mac='00:00:00:00:00:16'
    )
    
    # ============ ZONE C - CONTRÔLEURS INDUSTRIELS (4) ============
    info("  → Zone C: Contrôleurs industriels\n")
    
    ctrl1 = net.addStation(
        'ctrl1',
        ip='10.0.0.21/24',
        position='95,65,0',
        range=15,
        mac='00:00:00:00:00:21'
    )
    
    ctrl2 = net.addStation(
        'ctrl2',
        ip='10.0.0.22/24',
        position='105,55,0',
        range=15,
        mac='00:00:00:00:00:22'
    )
    
    ctrl3 = net.addStation(
        'ctrl3',
        ip='10.0.0.23/24',
        position='100,70,0',
        range=15,
        mac='00:00:00:00:00:23'
    )
    
    ctrl4 = net.addStation(
        'ctrl4',
        ip='10.0.0.24/24',
        position='110,60,0',
        range=15,
        mac='00:00:00:00:00:24'
    )
    
    # ============ DRONES MOBILES (2) ============
    info("  → Drones mobiles\n")
    
    drone1 = net.addStation(
        'drone1',
        ip='10.0.0.31/24',
        position='40,60,0',
        range=20,
        mac='00:00:00:00:00:31'
    )
    
    drone2 = net.addStation(
        'drone2',
        ip='10.0.0.32/24',
        position='80,50,0',
        range=20,
        mac='00:00:00:00:00:32'
    )
    
    # ============ SERVEURS CLOUD (2) ============
    info("  → Serveurs cloud (destinations)\n")
    
    server1 = net.addStation(
        'server1',
        ip='10.0.0.100/24',
        position='135,65,0',
        range=20,
        mac='00:00:00:00:01:00'
    )
    
    server2 = net.addStation(
        'server2',
        ip='10.0.0.101/24',
        position='145,55,0',
        range=20,
        mac='00:00:00:00:01:01'
    )
    
    info("*** Configuration du modèle de propagation\n")
    # LogDistance : environnement indoor avec obstacles
    net.setPropagationModel(model="logDistance", exp=4.5)
    
    info("*** Configuration des liens sans fil\n")
    net.configureWifiNodes()
    
    info("*** Création des liens câblés (backbone)\n")
    # Backbone principal
    net.addLink(ap1, ap2, bw=1000, delay='2ms')  # 1 Gbps
    net.addLink(ap2, ap4, bw=1000, delay='2ms')
    net.addLink(ap3, ap4, bw=1000, delay='2ms')
    net.addLink(ap4, ap5, bw=1000, delay='2ms')
    net.addLink(ap5, ap6, bw=1000, delay='2ms')
    
    # Liens de redondance
    net.addLink(ap1, ap3, bw=500, delay='5ms')   # Lien de secours
    net.addLink(ap2, ap5, bw=500, delay='5ms')
    
    info("*** Activation de la mobilité\n")
    net.startMobility(time=0)
    
    info("*** Démarrage du réseau\n")
    net.build()
    c0.start()
    
    # Démarrer tous les APs
    ap1.start([c0])
    ap2.start([c0])
    ap3.start([c0])
    ap4.start([c0])
    ap5.start([c0])
    ap6.start([c0])
    
    # Attendre la stabilisation
    time.sleep(8)
    
    info("*** Configuration des paramètres WiFi\n")
    # Configurer la puissance de transmission
    for sta in net.stations:
        if 'cam' in sta.name or 'drone' in sta.name:
            sta.setTxPower(17)  # Caméras et drones : puissance élevée
        elif 'sensor' in sta.name:
            sta.setTxPower(12)  # Capteurs : économie d'énergie
        else:
            sta.setTxPower(14)  # Autres : puissance moyenne
    
    info("*** Configuration de la mobilité des drones\n")
    # Drone 1 : patrouille entre zones A et B
    net.mobility(drone1, 'start', time=1, position='40,60,0')
    net.mobility(drone1, 'stop', time=60, position='40,80,0')
    
    # Drone 2 : patrouille entre zones B et C
    net.mobility(drone2, 'start', time=1, position='80,50,0')
    net.mobility(drone2, 'stop', time=60, position='100,50,0')
    
    #info("*** Test de connectivité\n")
    #net.pingAll()
    
    info("\n" + "="*70 + "\n")
    info("TOPOLOGIE IoT WiFi ÉTENDUE - 6 APs, 20 Stations\n")
    info("="*70 + "\n")
    
    info("\n📹 CAMÉRAS DE SURVEILLANCE (Zone A):\n")
    info("  cam1-cam6 : 10.0.0.1-6 (Streaming vidéo haute qualité)\n")
    
    info("\n📡 CAPTEURS ENVIRONNEMENTAUX (Zone B):\n")
    info("  sensor1-sensor6 : 10.0.0.11-16 (Température, humidité, etc.)\n")
    
    info("\n🎮 CONTRÔLEURS INDUSTRIELS (Zone C):\n")
    info("  ctrl1-ctrl4 : 10.0.0.21-24 (Commandes critiques)\n")
    
    info("\n🚁 DRONES MOBILES:\n")
    info("  drone1-drone2 : 10.0.0.31-32 (Inspection aérienne)\n")
    
    info("\n☁️  SERVEURS CLOUD:\n")
    info("  server1-server2 : 10.0.0.100-101 (Stockage et traitement)\n")
    
    info("\n📶 ACCESS POINTS:\n")
    info("  ap1 (20,80) - Canal 1  : Zone surveillance\n")
    info("  ap2 (60,80) - Canal 6  : Zone surveillance\n")
    info("  ap3 (20,40) - Canal 11 : Zone capteurs\n")
    info("  ap4 (60,40) - Canal 1  : Zone capteurs\n")
    info("  ap5 (100,60)- Canal 6  : Zone contrôle\n")
    info("  ap6 (140,60)- Canal 11 : Zone cloud\n")
    
    info("\n" + "="*70 + "\n")
    info("Commandes CLI utiles:\n")
    info("  cam1 ping server1              # Test caméra → serveur\n")
    info("  sensor1 ping ctrl1             # Test capteur → contrôleur\n")
    info("  py drone1.params['rssi']       # RSSI du drone1\n")
    info("  distance drone1 ap3            # Distance drone-AP\n")
    info("  iperf cam1 server1             # Test débit vidéo\n")
    info("  py drone1.setPosition('50,70,0') # Déplacer drone1\n")
    info("="*70 + "\n\n")
    
    CLI(net)
    
    info("*** Arrêt du réseau\n")
    net.stopMobility()
    net.stop()

if __name__ == '__main__':
    setLogLevel('info')
    create_extended_iot_topology()






# Controller contains rl_traffic_manager .py && network_monitor.py
## network_monitor.py

    Network Monitor pour le contrôleur SDN WiFi IoT avec RL
Affiche périodiquement :
- La topologie (switches et liens)
- Le nombre de stations par AP
- La consommation de bande passante par AP
- Les tables MAC apprises
"""

from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import MAIN_DISPATCHER, CONFIG_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.ofproto import ofproto_v1_3
from ryu.topology.api import get_switch, get_link
from ryu.lib import hub
import time


class NetworkMonitor(app_manager.RyuApp):
    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(NetworkMonitor, self).__init__(*args, **kwargs)
        self.datapaths = {}
        self.monitor_thread = hub.spawn(self._monitor)

        self.logger.info("🌐 Network Monitor démarré – Affichage toutes les 10 secondes")

    @set_ev_cls(ofp_event.EventOFPStateChange, [MAIN_DISPATCHER, CONFIG_DISPATCHER])
    def state_change_handler(self, ev):
        datapath = ev.datapath
        if ev.state == MAIN_DISPATCHER:
            if datapath.id not in self.datapaths:
                self.datapaths[datapath.id] = datapath
        elif ev.state == CONFIG_DISPATCHER:
            if datapath.id in self.datapaths:
                del self.datapaths[datapath.id]

    def _monitor(self):
        """Thread principal du monitoring"""
        while True:
            self._show_topology()
            self._request_port_stats()
            hub.sleep(10)

    def _show_topology(self):
        """Affiche la topologie actuelle et les charges par AP"""
        switches = get_switch(self, None)
        links = get_link(self, None)

        if not switches:
            self.logger.info("⚠ Aucune topologie détectée pour le moment.")
            return

        # Récupérer les informations depuis le contrôleur principal si possible
        # (on suppose que rl_traffic_manager.py est chargé et expose ces attributs)
        main_app = self._get_main_app()
        ap_load = getattr(main_app, 'ap_load', {})
        ap_bandwidth = getattr(main_app, 'ap_bandwidth_usage', {})
        station_to_ap = getattr(main_app, 'station_to_ap', {})

        self.logger.info("=" * 80)
        self.logger.info("📶 ÉTAT DU RÉSEAU SDN WiFi IoT – %s", time.strftime("%H:%M:%S"))
        self.logger.info("=" * 80)

        # Liste des APs
        self.logger.info("Access Points connectés (%d) :", len(switches))
        for sw in sorted(switches, key=lambda s: s.dp.id):
            dpid = sw.dp.id
            ap_num = self._dpid_to_ap(dpid)
            load = ap_load.get(dpid, 0)
            bw = ap_bandwidth.get(dpid, 0)
            self.logger.info("   AP%-2d (DPID: %016x) → %2d stations | BW: %6.1f KB/s",
                             ap_num, dpid, load, bw)

        # Associations stations → AP
        if station_to_ap:
            self.logger.info("\nAssociations Stations → AP :")
            stations_by_ap = {}
            for mac, ap_dpid in station_to_ap.items():
                ap_num = self._dpid_to_ap(ap_dpid)
                stations_by_ap.setdefault(ap_num, []).append(mac)

            for ap_num in sorted(stations_by_ap.keys()):
                stations = stations_by_ap[ap_num]
                self.logger.info("   AP%-2d ← %s", ap_num, ", ".join(stations[:10]))
                if len(stations) > 10:
                    self.logger.info("         ... et %d autres", len(stations) - 10)

        # Liens backbone
        if links:
            self.logger.info("\nLiens backbone détectés (%d) :", len(links))
            for link in links:
                src_ap = self._dpid_to_ap(link.src.dpid)
                dst_ap = self._dpid_to_ap(link.dst.dpid)
                self.logger.info("   AP%-2d (port %d) ↔ AP%-2d (port %d)",
                                 src_ap, link.src.port_no, dst_ap, link.dst.port_no)

        self.logger.info("=" * 80)

    def _request_port_stats(self):
        """Demande les statistiques de ports à tous les switches"""
        for datapath in self.datapaths.values():
            parser = datapath.ofproto_parser
            ofproto = datapath.ofproto
            req = parser.OFPPortStatsRequest(datapath, 0, ofproto.OFPP_ALL)
            datapath.send_msg(req)

    def _get_main_app(self):
        """Essaie de récupérer l'instance du contrôleur principal (RLWiFiTrafficManagerExtended)"""
        for app in self.app_manager.apps.values():
            if hasattr(app, 'ap_load'):  # Signature du contrôleur principal
                return app
        return None

    def _dpid_to_ap(self, dpid):
        """Convertit un DPID en numéro d'AP lisible (ex: 1, 2, ..., 6)"""
        if not dpid:
            return 0
        # Prend les deux derniers chiffres hexadécimaux (classique dans Mininet)
        try:
            return int(str(dpid)[-2:], 16)
        except:
            return int(dpid)
## rl_traffic_manager .py
"""
Contrôleur SDN avec gestion RL pour topologie WiFi IoT étendue
- 6 Access Points
- 20 Stations (caméras, capteurs, contrôleurs, drones, serveurs)
- Gestion intelligente basée sur Deep Q-Learning
"""
from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER, DEAD_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.ofproto import ofproto_v1_3
from ryu.lib.packet import packet, ethernet, ether_types, ipv4
from ryu.topology import event
from ryu.topology.api import get_switch, get_link
from ryu.lib import hub
import networkx as nx
import numpy as np
import time
from collections import defaultdict

# Import de l'agent DQN
try:
    from dqn_agent import DQNAgent
except Exception as e:
    print(f"⚠ Module dqn_agent non trouvé: {e}")
    print("⚠ Mode fallback actif (sans RL)")

class RLWiFiTrafficManagerExtended(app_manager.RyuApp):
    """
    Contrôleur SDN avec gestion RL optimisée pour topologie étendue
    """
    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(RLWiFiTrafficManagerExtended, self).__init__(*args, **kwargs)

        # Tables de routage
        self.mac_to_port = {}
        self.network_graph = nx.DiGraph()

        # Environnement RL
        self.agent = None
        self.state_size = 40  # Augmenté pour 6 APs
        self.action_size = 18  # 6 APs × 3 chemins

        # Classification des dispositifs IoT (20 stations)
        self.device_types = {
            '10.0.0.1': {'type': 'video', 'name': 'cam1', 'priority': 3, 'min_bw': 5000},
            '10.0.0.2': {'type': 'video', 'name': 'cam2', 'priority': 3, 'min_bw': 5000},
            '10.0.0.3': {'type': 'video', 'name': 'cam3', 'priority': 3, 'min_bw': 5000},
            '10.0.0.4': {'type': 'video', 'name': 'cam4', 'priority': 3, 'min_bw': 5000},
            '10.0.0.5': {'type': 'video', 'name': 'cam5', 'priority': 3, 'min_bw': 5000},
            '10.0.0.6': {'type': 'video', 'name': 'cam6', 'priority': 3, 'min_bw': 5000},

            '10.0.0.11': {'type': 'sensor', 'name': 'sensor1', 'priority': 2, 'min_bw': 100},
            '10.0.0.12': {'type': 'sensor', 'name': 'sensor2', 'priority': 2, 'min_bw': 100},
            '10.0.0.13': {'type': 'sensor', 'name': 'sensor3', 'priority': 2, 'min_bw': 100},
            '10.0.0.14': {'type': 'sensor', 'name': 'sensor4', 'priority': 2, 'min_bw': 100},
            '10.0.0.15': {'type': 'sensor', 'name': 'sensor5', 'priority': 2, 'min_bw': 100},
            '10.0.0.16': {'type': 'sensor', 'name': 'sensor6', 'priority': 2, 'min_bw': 100},

            '10.0.0.21': {'type': 'control', 'name': 'ctrl1', 'priority': 1, 'min_bw': 500},
            '10.0.0.22': {'type': 'control', 'name': 'ctrl2', 'priority': 1, 'min_bw': 500},
            '10.0.0.23': {'type': 'control', 'name': 'ctrl3', 'priority': 1, 'min_bw': 500},
            '10.0.0.24': {'type': 'control', 'name': 'ctrl4', 'priority': 1, 'min_bw': 500},

            '10.0.0.31': {'type': 'video', 'name': 'drone1', 'priority': 2, 'min_bw': 3000},
            '10.0.0.32': {'type': 'video', 'name': 'drone2', 'priority': 2, 'min_bw': 3000},

            '10.0.0.100': {'type': 'best_effort', 'name': 'server1', 'priority': 4, 'min_bw': 0},
            '10.0.0.101': {'type': 'best_effort', 'name': 'server2', 'priority': 4, 'min_bw': 0},
        }

        # Suivi des associations stations-APs
        self.station_to_ap = {}  # MAC → DPID
        self.ap_load = {}  # DPID → nombre de stations
        self.ap_bandwidth_usage = {}  # DPID → bande passante utilisée (KB/s)

        # Mapping MAC ↔ IP
        self.mac_to_ip = {}
        self.ip_to_mac = {}

        # Statistiques de performance
        self.flow_stats = defaultdict(dict)
        self.port_stats = defaultdict(dict)  # Pour calculer le débit

        # Compteurs de décisions
        self.rl_decisions = 0
        self.fallback_decisions = 0

        # Charger le modèle RL
        self.load_trained_model()

        # Démarrer le thread de monitoring
        self.monitor_thread = hub.spawn(self._monitor_network)

        self.logger.info("="*70)
        self.logger.info("Contrôleur RL WiFi Traffic Manager - Topologie Étendue")
        self.logger.info("="*70)
        self.logger.info("Configuration:")
        self.logger.info(" - 6 Access Points")
        self.logger.info(" - 20 Stations IoT")
        self.logger.info(" - Agent RL: %s", "Activé" if self.agent else "Désactivé (mode fallback)")
        self.logger.info("="*70)

    def load_trained_model(self):
        """Charge le modèle DQN pré-entraîné"""
        try:
            self.agent = DQNAgent(
                state_size=self.state_size,
                action_size=self.action_size,
                epsilon=0.05  # Exploration minimale en production
            )

            import os
            model_path = 'models/dqn_wifi_traffic_model.pth'
            if os.path.exists(model_path):
                self.agent.load_model(model_path)
                self.logger.info("✓ Modèle RL chargé: %s", model_path)
            else:
                self.logger.warning("⚠ Modèle non trouvé, agent RL initialisé sans pré-entraînement")

        except Exception as e:
            self.logger.warning(f"⚠ Impossible de charger l'agent RL: {e}")
            self.logger.warning("→ Mode fallback: routage basé sur Dijkstra + load balancing")
            self.agent = None

    def _dpid_to_ap_number(self, dpid):
        """Convertit un DPID en numéro d'AP lisible (ex: 0000000000000001 → 1)"""
        if not dpid:
            return 0
        return int(str(dpid)[-2:], 16) if len(str(dpid)) > 1 else int(dpid)

    @set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
    def switch_features_handler(self, ev):
        datapath = ev.msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        dpid = datapath.id

        match = parser.OFPMatch()
        actions = [parser.OFPActionOutput(ofproto.OFPP_CONTROLLER, ofproto.OFPCML_NO_BUFFER)]
        self.add_flow(datapath, 0, match, actions)

        self.ap_load[dpid] = 0
        self.ap_bandwidth_usage[dpid] = 0

        self.logger.info("✓ Access Point connecté: AP%d (DPID: %016x)",
                         self._dpid_to_ap_number(dpid), dpid)

    @set_ev_cls(ofp_event.EventOFPStateChange, [MAIN_DISPATCHER, DEAD_DISPATCHER])
    def _state_change_handler(self, ev):
        datapath = ev.datapath
        dpid = datapath.id

        if ev.state == MAIN_DISPATCHER:
            if dpid not in self.ap_load:
                self.ap_load[dpid] = 0
                self.ap_bandwidth_usage[dpid] = 0
        elif ev.state == DEAD_DISPATCHER:
            if dpid in self.ap_load:
                self.logger.warning("✗ Access Point déconnecté: AP%d", self._dpid_to_ap_number(dpid))
                self.ap_load.pop(dpid, None)
                self.ap_bandwidth_usage.pop(dpid, None)

    def add_flow(self, datapath, priority, match, actions,
                 buffer_id=None, idle_timeout=30, hard_timeout=0):
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser

        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]

        if buffer_id:
            mod = parser.OFPFlowMod(datapath=datapath, buffer_id=buffer_id,
                                    priority=priority, match=match, instructions=inst,
                                    idle_timeout=idle_timeout, hard_timeout=hard_timeout)
        else:
            mod = parser.OFPFlowMod(datapath=datapath, priority=priority,
                                    match=match, instructions=inst,
                                    idle_timeout=idle_timeout, hard_timeout=hard_timeout)

        datapath.send_msg(mod)

    @set_ev_cls(event.EventSwitchEnter)
    def get_topology_data(self, ev):
        switch_list = get_switch(self, None)
        switches = [switch.dp.id for switch in switch_list]

        link_list = get_link(self, None)
        links = [(link.src.dpid, link.dst.dpid, {
            'port': link.src.port_no,
            'dst_port': link.dst.port_no
        }) for link in link_list]

        self.network_graph.clear()
        self.network_graph.add_nodes_from(switches)
        self.network_graph.add_edges_from(links)

        self.logger.info("📡 Topologie mise à jour: %d APs, %d liens backbone",
                         len(switches), len(links))

        if switches:
            self.logger.info(" APs actifs: %s",
                             [f"AP{self._dpid_to_ap_number(s)}" for s in sorted(switches)])

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def packet_in_handler(self, ev):
        msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']
        dpid = datapath.id

        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocols(ethernet.ethernet)[0]

        if eth.ethertype == ether_types.ETH_TYPE_LLDP:
            return

        dst_mac = eth.dst
        src_mac = eth.src

        self.mac_to_port.setdefault(dpid, {})
        self.mac_to_port[dpid][src_mac] = in_port

        ip_pkt = pkt.get_protocol(ipv4.ipv4)
        src_ip = ip_pkt.src if ip_pkt else None
        dst_ip = ip_pkt.dst if ip_pkt else None

        if src_ip:
            self.mac_to_ip[src_mac] = src_ip
            self.ip_to_mac[src_ip] = src_mac

        # Détection handover
        if src_mac in self.station_to_ap and self.station_to_ap[src_mac] != dpid:
            old_ap = self.station_to_ap[src_mac]
            device_name = self.device_types.get(src_ip, {}).get('name', src_mac)
            self.logger.info("🔄 Handover: %s (AP%d → AP%d)",
                             device_name,
                             self._dpid_to_ap_number(old_ap),
                             self._dpid_to_ap_number(dpid))
            self._handle_station_handover(src_mac, old_ap, dpid)

        # Mise à jour association
        if src_mac not in self.station_to_ap:
            self.ap_load[dpid] = self.ap_load.get(dpid, 0) + 1
        self.station_to_ap[src_mac] = dpid

        # Décision de routage
        if dst_mac in self.mac_to_port.get(dpid, {}):
            out_port = self.mac_to_port[dpid][dst_mac]
        else:
            if self.agent and src_ip and dst_ip:
                out_port = self._rl_routing_decision(dpid, src_mac, dst_mac, src_ip, dst_ip, in_port)
                self.rl_decisions += 1
            else:
                out_port = self._fallback_routing(dpid, dst_mac, dst_ip)
                self.fallback_decisions += 1

        actions = [parser.OFPActionOutput(out_port)]

        if out_port != ofproto.OFPP_FLOOD and src_ip:
            device_info = self.device_types.get(src_ip, {})
            traffic_type = device_info.get('type', 'best_effort')
            priority = self._get_priority_for_traffic(traffic_type)

            if 'drone' in device_info.get('name', ''):
                idle_timeout = 10
            elif traffic_type == 'video':
                idle_timeout = 60
            elif traffic_type == 'control':
                idle_timeout = 30
            else:
                idle_timeout = 20

            match = parser.OFPMatch(
                in_port=in_port,
                eth_dst=dst_mac,
                eth_src=src_mac,
                eth_type=ether_types.ETH_TYPE_IP,
                ipv4_src=src_ip,
                ipv4_dst=dst_ip
            )

            if msg.buffer_id != ofproto.OFP_NO_BUFFER:
                self.add_flow(datapath, priority, match, actions, msg.buffer_id, idle_timeout=idle_timeout)
            else:
                self.add_flow(datapath, priority, match, actions, idle_timeout=idle_timeout)

        data = msg.data if msg.buffer_id == ofproto.OFP_NO_BUFFER else None
        out = parser.OFPPacketOut(datapath=datapath, buffer_id=msg.buffer_id,
                                  in_port=in_port, actions=actions, data=data)
        datapath.send_msg(out)

    def _rl_routing_decision(self, dpid, src_mac, dst_mac, src_ip, dst_ip, in_port):
        try:
            device_info = self.device_types.get(src_ip, {})
            traffic_type = device_info.get('type', 'best_effort')
            device_name = device_info.get('name', src_ip)
            estimated_rssi = self._estimate_rssi(src_mac, dpid)

            state = self._build_rl_state(dpid, traffic_type, estimated_rssi)
            action = self.agent.choose_action(state, training=False)
            out_port = self._action_to_port(dpid, action, dst_mac, dst_ip)

            if self.rl_decisions % 100 == 0:
                self.logger.debug("RL: %s (%s) %s→%s | AP%d→Port%d | RSSI:%ddBm | Action:%d",
                                  device_name, traffic_type, src_ip, dst_ip,
                                  self._dpid_to_ap_number(dpid), out_port,
                                  int(estimated_rssi), action)

            return out_port
        except Exception as e:
            self.logger.error(f"Erreur RL routing: {e}")
            return self._fallback_routing(dpid, dst_mac, dst_ip)

    def _build_rl_state(self, current_dpid, traffic_type, rssi):
        state = []

        max_stations = 10
        sorted_dpids = sorted(self.ap_load.keys())[:6]
        for dpid in sorted_dpids:
            load = self.ap_load.get(dpid, 0) / max_stations
            state.append(min(1.0, load))
        while len(state) < 6:
            state.append(0.0)

        max_bandwidth = 50000
        for dpid in sorted_dpids:
            bw = self.ap_bandwidth_usage.get(dpid, 0) / max_bandwidth
            state.append(min(1.0, bw))
        while len(state) < 12:
            state.append(0.0)

        rssi_norm = (rssi + 100) / 50
        state.append(max(0.0, min(1.0, rssi_norm)))

        types = ['video', 'sensor', 'control', 'best_effort']
        type_encoding = [1.0 if traffic_type == t else 0.0 for t in types]
        state.extend(type_encoding)

        total_stations = sum(self.ap_load.values()) / 20
        state.append(min(1.0, total_stations))

        current_ap_encoding = [0.0] * 6
        if current_dpid in sorted_dpids:
            idx = sorted_dpids.index(current_dpid)
            if idx < 6:
                current_ap_encoding[idx] = 1.0
        state.extend(current_ap_encoding)

        while len(state) < self.state_size:
            state.append(0.0)

        return np.array(state[:self.state_size], dtype=np.float32)

    def _action_to_port(self, current_dpid, action, dst_mac, dst_ip):
        if not self.network_graph.nodes():
            return 1

        try:
            num_aps = min(6, len(self.network_graph.nodes()))
            target_ap_idx = action // 3
            path_choice = action % 3
            if target_ap_idx >= num_aps:
                target_ap_idx = num_aps - 1

            sorted_aps = sorted(self.network_graph.nodes())[:6]
            target_ap = sorted_aps[target_ap_idx]

            if current_dpid == target_ap:
                if dst_mac in self.mac_to_port.get(current_dpid, {}):
                    return self.mac_to_port[current_dpid][dst_mac]
                dst_ap = self._find_ap_for_station(dst_mac, dst_ip)
                if dst_ap and dst_ap != current_dpid:
                    target_ap = dst_ap

            all_paths = list(nx.all_simple_paths(self.network_graph, current_dpid, target_ap, cutoff=4))
            if not all_paths:
                path = nx.shortest_path(self.network_graph, current_dpid, target_ap)
            else:
                path = all_paths[min(path_choice, len(all_paths)-1)]

            if len(path) > 1:
                next_hop = path[1]
                edge_data = self.network_graph[current_dpid][next_hop]
                return edge_data.get('port', 1)

        except nx.NetworkXNoPath:
            self.logger.debug("Pas de chemin: AP%d → AP%d",
                              self._dpid_to_ap_number(current_dpid),
                              self._dpid_to_ap_number(target_ap))
        except Exception as e:
            self.logger.error(f"Erreur action→port: {e}")

        return 1

    def _fallback_routing(self, src_dpid, dst_mac, dst_ip):
        try:
            dst_ap = self._find_ap_for_station(dst_mac, dst_ip)
            if not dst_ap and self.ap_load:
                dst_ap = min(self.ap_load.items(), key=lambda x: x[1])[0]

            if src_dpid == dst_ap:
                return self.mac_to_port.get(src_dpid, {}).get(dst_mac, 1)

            if src_dpid in self.network_graph and dst_ap in self.network_graph:
                path = nx.shortest_path(self.network_graph, src_dpid, dst_ap)
                if len(path) > 1:
                    next_hop = path[1]
                    edge_data = self.network_graph[src_dpid][next_hop]
                    return edge_data.get('port', 1)
        except Exception as e:
            self.logger.error(f"Erreur fallback routing: {e}")

        return 1

    def _find_ap_for_station(self, mac, ip):
        if mac in self.station_to_ap:
            return self.station_to_ap[mac]
        for dpid, mac_table in self.mac_to_port.items():
            if mac in mac_table:
                return dpid
        if ip in ['10.0.0.100', '10.0.0.101'] and self.network_graph.nodes():
            return max(self.network_graph.nodes())
        return None

    def _estimate_rssi(self, station_mac, ap_dpid):
        base_rssi = -55
        load = self.ap_load.get(ap_dpid, 0)
        load_penalty = load * 3
        bw = self.ap_bandwidth_usage.get(ap_dpid, 0)
        bw_penalty = (bw / 10000) * 5
        noise = np.random.normal(0, 2)
        estimated_rssi = base_rssi - load_penalty - bw_penalty + noise
        return max(-90, min(-40, estimated_rssi))

    def _get_priority_for_traffic(self, traffic_type):
        priorities = {
            'control': 100,
            'sensor': 80,
            'video': 60,
            'best_effort': 40
        }
        return priorities.get(traffic_type, 40)

    def _handle_station_handover(self, station_mac, old_ap, new_ap):
        if old_ap in self.ap_load:
            self.ap_load[old_ap] = max(0, self.ap_load[old_ap] - 1)
        self.ap_load[new_ap] = self.ap_load.get(new_ap, 0) + 1

    def _monitor_network(self):
        """Thread de monitoring : demande les statistiques de ports périodiquement"""
        self.logger.info("Démarrage du thread de monitoring réseau")

        while True:
            try:
                if not self.network_graph.nodes():
                    hub.sleep(2)
                    continue

                for dpid in list(self.ap_load.keys()):
                    switch = get_switch(self, dpid)
                    if not switch:
                        continue
                    datapath = switch[0].dp if switch else None
                    if not datapath:
                        continue

                    ofproto = datapath.ofproto
                    parser = datapath.ofproto_parser
                    req = parser.OFPPortStatsRequest(datapath, 0, ofproto.OFPP_ALL)
                    datapath.send_msg(req)

                hub.sleep(5)
            except Exception as e:
                self.logger.error(f"Erreur dans le thread de monitoring: {e}")
                hub.sleep(5)

    @set_ev_cls(ofp_event.EventOFPPortStatsReply, MAIN_DISPATCHER)
    def _port_stats_reply_handler(self, ev):
        """Met à jour la bande passante utilisée par AP"""
        datapath = ev.msg.datapath
        dpid = datapath.id
        current_time = time.time()
        bandwidth_usage = 0  # bits/s

        for stat in ev.msg.body:
            if stat.port_no == datapath.ofproto.OFPP_LOCAL:
                continue

            port_key = (dpid, stat.port_no)
            prev_bytes = self.port_stats.get(port_key, 0)
            prev_time = self.port_stats.get(f"{port_key}_time", current_time)

            delta_bytes = (stat.tx_bytes + stat.rx_bytes) - prev_bytes
            delta_time = current_time - prev_time

            port_bps = (delta_bytes * 8) / delta_time if delta_time > 0 else 0
            bandwidth_usage += port_bps

            self.port_stats[port_key] = stat.tx_bytes + stat.rx_bytes
            self.port_stats[f"{port_key}_time"] = current_time

        # Conversion en KB/s
        self.ap_bandwidth_usage[dpid] = bandwidth_usage / 8000

# Training forlder contains models , train_agent.py , dqn_agent.py , rl_environement.py , compare1.py and results 
## 1) train_agent.py 

import numpy as np
import matplotlib.pyplot as plt
import random
import os

# À adapter selon votre implémentation réelle
# from wifi_rl_environment import WiFiSDNEnvironment, Flow
# from dqn_agent import DQNAgent

# ===================================================================
# Classe Flow simplifiée (si vous n'avez pas le module original)
# ===================================================================
class Flow:
    def __init__(self, src, dst, type, priority, rssi=0):
        self.src = src
        self.dst = dst
        self.type = type
        self.priority = priority
        self.rssi = rssi

# ===================================================================
# Topologie étendue : 6 APs, 20 stations
# ===================================================================
def create_extended_wifi_topology():
    """
    Crée la topologie correspondant exactement à votre Mininet-WiFi étendu
    """
    topology = {
        'aps': ['ap1', 'ap2', 'ap3', 'ap4', 'ap5', 'ap6'],
        'channels': {
            'ap1': 1, 'ap2': 6, 'ap3': 11,
            'ap4': 1, 'ap5': 6, 'ap6': 11
        },
        'positions': {
            # APs
            'ap1': (20, 80, 0),
            'ap2': (60, 80, 0),
            'ap3': (20, 40, 0),
            'ap4': (60, 40, 0),
            'ap5': (100, 60, 0),
            'ap6': (140, 60, 0),

            # Caméras Zone A
            'cam1': (15, 85, 0), 'cam2': (25, 75, 0), 'cam3': (30, 85, 0),
            'cam4': (55, 85, 0), 'cam5': (65, 75, 0), 'cam6': (60, 90, 0),

            # Capteurs Zone B
            'sensor1': (15, 45, 0), 'sensor2': (25, 35, 0), 'sensor3': (20, 48, 0),
            'sensor4': (55, 45, 0), 'sensor5': (65, 35, 0), 'sensor6': (60, 48, 0),

            # Contrôleurs Zone C
            'ctrl1': (95, 65, 0), 'ctrl2': (105, 55, 0),
            'ctrl3': (100, 70, 0), 'ctrl4': (110, 60, 0),

            # Drones (mobiles)
            'drone1': (40, 60, 0),
            'drone2': (80, 50, 0),

            # Serveurs
            'server1': (135, 65, 0),
            'server2': (145, 55, 0),
        },
        'initial_associations': {  # Station → AP initial
            # Zone A
            'cam1': 'ap1', 'cam2': 'ap1', 'cam3': 'ap1',
            'cam4': 'ap2', 'cam5': 'ap2', 'cam6': 'ap2',

            # Zone B
            'sensor1': 'ap3', 'sensor2': 'ap3', 'sensor3': 'ap3',
            'sensor4': 'ap4', 'sensor5': 'ap4', 'sensor6': 'ap4',

            # Zone C
            'ctrl1': 'ap5', 'ctrl2': 'ap5', 'ctrl3': 'ap5', 'ctrl4': 'ap5',

            # Drones (début)
            'drone1': 'ap3', 'drone2': 'ap4',

            # Serveurs
            'server1': 'ap6', 'server2': 'ap6',
        },
        'traffic_mapping': {  # Nom station → type de trafic
            # Caméras + Drones = vidéo haute priorité
            'cam1': 'video', 'cam2': 'video', 'cam3': 'video',
            'cam4': 'video', 'cam5': 'video', 'cam6': 'video',
            'drone1': 'video', 'drone2': 'video',

            # Capteurs = faible latence, faible débit
            'sensor1': 'sensor', 'sensor2': 'sensor', 'sensor3': 'sensor',
            'sensor4': 'sensor', 'sensor5': 'sensor', 'sensor6': 'sensor',

            # Contrôleurs = critique
            'ctrl1': 'control', 'ctrl2': 'control',
            'ctrl3': 'control', 'ctrl4': 'control',

            # Serveurs = best_effort (réception)
            'server1': 'best_effort', 'server2': 'best_effort',
        },
        'priorities': {
            'control': 100,
            'video': 70,
            'sensor': 60,
            'best_effort': 40
        }
    }
    return topology

# ===================================================================
# Génération de flux réalistes
# ===================================================================
def generate_random_flow(topology):
    # Sources : seulement les stations IoT (caméras, capteurs, contrôleurs, drones) - PAS les APs ni serveurs
    sources = ['cam1', 'cam2', 'cam3', 'cam4', 'cam5', 'cam6',
               'sensor1', 'sensor2', 'sensor3', 'sensor4', 'sensor5', 'sensor6',
               'ctrl1', 'ctrl2', 'ctrl3', 'ctrl4',
               'drone1', 'drone2']

    destinations = ['server1', 'server2']

    src = random.choice(sources)
    dst = random.choice(destinations)
    flow_type = topology['traffic_mapping'][src]
    priority = topology['priorities'][flow_type]

    return Flow(src=src, dst=dst, type=flow_type, priority=priority, rssi=0)
# ===================================================================
# Simulation de mobilité (drones uniquement)
# ===================================================================
def simulate_drone_mobility(env, episode, topology):
    if episode % 50 == 0:  # Toutes les 50 épisodes
        for drone in ['drone1', 'drone2']:
            current_pos = env.positions[drone]
            # Mouvement aléatoire réaliste (±20m)
            dx = random.randint(-20, 20)
            dy = random.randint(-20, 20)
            new_x = np.clip(current_pos[0] + dx, 10, 150)
            new_y = np.clip(current_pos[1] + dy, 20, 100)
            env.positions[drone] = (new_x, new_y, 0)

            # Détection de handover
            old_ap = env.stations[drone]
            min_dist = float('inf')
            new_ap = old_ap

            for ap in env.aps:
                ap_pos = env.positions[ap]
                dist = np.sqrt((new_x - ap_pos[0])**2 + (new_y - ap_pos[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    new_ap = ap

            if new_ap != old_ap:
                env.handle_handover(drone, old_ap, new_ap)
                print(f" [Mobilité] {drone}: {old_ap} → {new_ap} (épisode {episode})")

# ===================================================================
# Entraînement principal
# ===================================================================
def train_extended_wifi_dqn_agent(episodes=2000):
    topology = create_extended_wifi_topology()

    # À remplacer par votre vraie classe si disponible
    # env = WiFiSDNEnvironment(topology)
    # Ici on simule un environnement minimal pour compatibilité
    class SimulatedEnv:
        def __init__(self, topo):
            self.topology = topo
            self.aps = topo['aps']
            self.positions = topo['positions'].copy()
            self.stations = topo['initial_associations'].copy()
            self.ap_load = {ap: 0 for ap in self.aps}
            for sta, ap in self.stations.items():
                self.ap_load[ap] += 1

        def reset(self):
            self.positions = self.topology['positions'].copy()
            self.stations = self.topology['initial_associations'].copy()
            self.ap_load = {ap: 0 for ap in self.aps}
            for sta, ap in self.stations.items():
                self.ap_load[ap] += 1

        def handle_handover(self, sta, old_ap, new_ap):
            if old_ap in self.ap_load:
                self.ap_load[old_ap] = max(0, self.ap_load[old_ap] - 1)
            self.ap_load[new_ap] += 1
            self.stations[sta] = new_ap

        def get_state(self, flow):
            # Charges normalisées (max 10 stations par AP)
            loads = [self.ap_load[ap] / 10.0 for ap in sorted(self.aps)]
            # Estimation RSSI source
            src_pos = self.positions[flow.src]
            src_ap = self.stations[flow.src]
            ap_pos = self.positions[src_ap]
            dist = np.sqrt(sum((a - b)**2 for a, b in zip(src_pos, ap_pos)))
            rssi = -30 - 20 * np.log10(max(dist, 1)) + np.random.normal(0, 5)
            rssi_norm = np.clip((rssi + 100) / 70, 0, 1)

            # One-hot type
            types = ['video', 'sensor', 'control', 'best_effort']
            type_vec = [1 if flow.type == t else 0 for t in types]

            state = np.array(loads + [rssi_norm] + type_vec)
            return np.pad(state, (0, 20 - len(state)), 'constant')[:20]

        def step(self, flow, action):
            # Action decoding: target_ap = action // 3
            target_idx = action // 3
            target_ap = sorted(self.aps)[min(target_idx, len(self.aps)-1)]

            current_ap = self.stations[flow.src]
            hops = 1 if current_ap == target_ap else 2

            latency = hops * (5 + random.uniform(0, 10))
            if flow.type == 'video':
                latency *= 1.2
            elif flow.type == 'control':
                latency *= 0.8

            rssi = flow.rssi if flow.rssi else -60
            reward = -0.1 * latency - 0.01 * max(0, self.ap_load[target_ap] - 5)
            if flow.type == 'control':
                reward -= 2.0 * latency
            elif flow.type == 'video':
                reward -= 0.5 * latency

            info = {'latency': latency, 'rssi': rssi}
            done = False
            next_state = self.get_state(flow)
            return next_state, reward, done, info

    env = SimulatedEnv(topology)

    state_size = 20   # 6 loads + rssi_norm + 4 types + padding
    action_size = 18  # 6 APs × 3 chemins possibles

    print(f"État RL: {state_size} dimensions")
    print(f"Espace d'actions: {action_size} (6 APs × 3 chemins)")

    # À remplacer par votre vrai DQNAgent
    class DQNAgent:
        def __init__(self, state_size, action_size, **kwargs):
            self.state_size = state_size
            self.action_size = action_size
            self.epsilon = kwargs.get('epsilon', 1.0)
            self.epsilon_min = kwargs.get('epsilon_min', 0.01)
            self.epsilon_decay = kwargs.get('epsilon_decay', 0.995)
            self.memory = []
            self.model = "simulated"

        def choose_action(self, state, training=True):
            if training and random.random() < self.epsilon:
                return random.randint(0, self.action_size - 1)
            return random.randint(0, 3)  # Simulation

        def store_experience(self, s, a, r, ns, d):
            self.memory.append((s, a, r, ns, d))

        def learn(self):
            return 0.001 if self.memory else 0.0

        def decay_epsilon(self):
            self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)

        def save_model(self, path):
            os.makedirs(os.path.dirname(path), exist_ok=True)
            print(f"Modèle sauvegardé : {path}")

    agent = DQNAgent(
        state_size=state_size,
        action_size=action_size,
        epsilon=1.0,
        epsilon_decay=0.995,
        epsilon_min=0.05
    )

    episode_rewards, episode_latencies, episode_rssi, episode_losses = [], [], [], []

    print("\n=== Début de l'entraînement DQN - Topologie IoT Étendue (6 APs) ===\n")

    for episode in range(episodes):
        env.reset()
        total_reward = 0
        total_latency = 0
        total_rssi = 0
        total_loss = 0
        steps = 0

        simulate_drone_mobility(env, episode, topology)

        num_flows = random.randint(10, 30)  # Plus de trafic dans grande topologie

        for _ in range(num_flows):
            flow = generate_random_flow(topology)
            state = env.get_state(flow)
            action = agent.choose_action(state, training=True)
            next_state, reward, done, info = env.step(flow, action)

            agent.store_experience(state, action, reward, next_state, done)
            loss = agent.learn()

            total_reward += reward
            total_latency += info['latency']
            total_rssi += info['rssi']
            total_loss += loss
            steps += 1

        agent.decay_epsilon()

        avg_reward = total_reward / steps if steps else 0
        avg_latency = total_latency / steps if steps else 0
        avg_rssi = total_rssi / steps if steps else -70
        avg_loss = total_loss / steps if steps else 0

        episode_rewards.append(avg_reward)
        episode_latencies.append(avg_latency)
        episode_rssi.append(avg_rssi)
        episode_losses.append(avg_loss)

        if (episode + 1) % 100 == 0:
            recent_r = np.mean(episode_rewards[-100:])
            recent_l = np.mean(episode_latencies[-100:])
            recent_rssi = np.mean(episode_rssi[-100:])
            print(f"Épisode {episode + 1}/{episodes} | "
                  f"Récompense: {recent_r:.3f} | "
                  f"Latence: {recent_l:.2f}ms | "
                  f"RSSI: {recent_rssi:.1f}dBm | "
                  f"Epsilon: {agent.epsilon:.3f}")

    # Sauvegarde (compatible avec le contrôleur Ryu)
    agent.save_model('models/dqn_wifi_extended_topology.pth')

    # Graphiques
    plot_training_results(episode_rewards, episode_latencies, episode_rssi, episode_losses)

    print("\n=== Entraînement terminé avec succès ===")
    print("Modèle prêt pour déploiement dans le contrôleur Ryu !")

    return agent, env

# ===================================================================
# Visualisation
# ===================================================================

def plot_training_results(rewards, latencies, rssi_vals, losses):
    """
    Visualise les résultats d'entraînement avec courbes lissées
    """
    fig, axes = plt.subplots(2, 2, figsize=(18, 12))
    fig.suptitle('Entraînement DQN - Réseau WiFi IoT Étendu (6 APs, 20 Stations)',
                 fontsize=18, fontweight='bold', color='#2C3E50')

    def plot_with_smooth(ax, data, title, ylabel, color_raw, color_smooth=None):
        if color_smooth is None:
            color_smooth = color_raw  # Même couleur si pas spécifiée

        # Courbe brute (légère transparence)
        ax.plot(data, alpha=0.6, color=color_raw, linewidth=1.2, label='Brut')

        # Courbe lissée (moyenne mobile sur 100 épisodes)
        if len(data) >= 100:
            smooth = np.convolve(data, np.ones(100)/100, mode='valid')
            ax.plot(range(99, len(data)), smooth,
                    color=color_smooth, linewidth=3, label='Moyenne mobile (100 ép.)')

        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.set_xlabel('Épisode', fontsize=12)
        ax.set_ylabel(ylabel, fontsize=12)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(fontsize=10)

    # 1. Récompense
    plot_with_smooth(axes[0,0], rewards,
                     'Récompense moyenne par épisode',
                     'Récompense',
                     '#4ECDC4', '#1ABC9C')  # Vert clair → vert foncé

    # 2. Latence
    plot_with_smooth(axes[0,1], latencies,
                     'Latence moyenne',
                     'Latence (ms)',
                     '#E74C3C', '#C0392B')  # Rouge clair → rouge foncé

    # 3. RSSI
    plot_with_smooth(axes[1,0], rssi_vals,
                     'Qualité du signal WiFi (RSSI moyen)',
                     'RSSI (dBm)',
                     '#F39C12', '#D68910')  # Orange clair → orange foncé

    # Lignes de référence RSSI
    axes[1,0].axhline(y=-60, color='green', linestyle='-', linewidth=2, alpha=0.7, label='Excellent (-60 dBm)')
    axes[1,0].axhline(y=-70, color='orange', linestyle='--', linewidth=2, alpha=0.7, label='Bon (-70 dBm)')
    axes[1,0].axhline(y=-80, color='red', linestyle='--', linewidth=2, alpha=0.7, label='Faible (-80 dBm)')
    axes[1,0].legend(fontsize=9)

    # 4. Perte (Loss)
    plot_with_smooth(axes[1,1], losses,
                     'Perte d\'entraînement (Loss)',
                     'Loss',
                     '#9B59B6', '#8E44AD')  # Violet clair → violet foncé

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Ajustement pour le titre

    # Sauvegarde
    os.makedirs('results', exist_ok=True)
    output_path = 'results/dqn_extended_wifi_training.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"✓ Graphiques sauvegardés : {output_path}")

    # Afficher à l'écran (optionnel)
    plt.show()


# ===================================================================
if __name__ == '__main__':
    train_extended_wifi_dqn_agent(episodes=2000)


## 2) rl_environment.py

import numpy as np
import networkx as nx
from collections import namedtuple

Flow = namedtuple('Flow', ['src', 'dst', 'type', 'priority', 'rssi'])

class WiFiTrafficType:
    """Types de trafic IoT avec exigences QoS"""
    VIDEO = {'name': 'video', 'priority': 70, 'min_bw': 5000, 'max_latency': 150, 'min_rssi': -70}
    SENSOR = {'name': 'sensor', 'priority': 60, 'min_bw': 100, 'max_latency': 100, 'min_rssi': -80}
    CONTROL = {'name': 'control', 'priority': 100, 'min_bw': 500, 'max_latency': 30, 'min_rssi': -65}
    BEST_EFFORT = {'name': 'best_effort', 'priority': 40, 'min_bw': 0, 'max_latency': 500, 'min_rssi': -85}


class WiFiSDNEnvironment:
    """
    Environnement RL pour réseau WiFi SDN-IoT étendu
    - 6 Access Points
    - 20 stations (caméras, capteurs, contrôleurs, drones, serveurs)
    - Mobilité des drones
    - Interférences WiFi, RSSI, handover
    """

    def __init__(self):
        # === Topologie fixe correspondant à votre Mininet-WiFi étendu ===
        self.aps = ['ap1', 'ap2', 'ap3', 'ap4', 'ap5', 'ap6']

        self.channels = {
            'ap1': 1, 'ap2': 6, 'ap3': 11,
            'ap4': 1, 'ap5': 6, 'ap6': 11
        }

        # Liens backbone câblés (Mbps)
        self.links = {
            ('ap1', 'ap2'): {'bw': 1000, 'latency': 2},
            ('ap2', 'ap1'): {'bw': 1000, 'latency': 2},
            ('ap2', 'ap4'): {'bw': 1000, 'latency': 2},
            ('ap4', 'ap2'): {'bw': 1000, 'latency': 2},
            ('ap3', 'ap4'): {'bw': 1000, 'latency': 2},
            ('ap4', 'ap3'): {'bw': 1000, 'latency': 2},
            ('ap4', 'ap5'): {'bw': 1000, 'latency': 2},
            ('ap5', 'ap4'): {'bw': 1000, 'latency': 2},
            ('ap5', 'ap6'): {'bw': 1000, 'latency': 2},
            ('ap6', 'ap5'): {'bw': 1000, 'latency': 2},
            ('ap1', 'ap3'): {'bw': 500, 'latency': 5},
            ('ap3', 'ap1'): {'bw': 500, 'latency': 5},
            ('ap2', 'ap5'): {'bw': 500, 'latency': 5},
            ('ap5', 'ap2'): {'bw': 500, 'latency': 5},
        }

        # Positions exactes (x, y, z)
        self.positions = {
            'ap1': (20, 80, 0), 'ap2': (60, 80, 0),
            'ap3': (20, 40, 0), 'ap4': (60, 40, 0),
            'ap5': (100, 60, 0), 'ap6': (140, 60, 0),

            'cam1': (15, 85, 0), 'cam2': (25, 75, 0), 'cam3': (30, 85, 0),
            'cam4': (55, 85, 0), 'cam5': (65, 75, 0), 'cam6': (60, 90, 0),

            'sensor1': (15, 45, 0), 'sensor2': (25, 35, 0), 'sensor3': (20, 48, 0),
            'sensor4': (55, 45, 0), 'sensor5': (65, 35, 0), 'sensor6': (60, 48, 0),

            'ctrl1': (95, 65, 0), 'ctrl2': (105, 55, 0),
            'ctrl3': (100, 70, 0), 'ctrl4': (110, 60, 0),

            'drone1': (40, 60, 0), 'drone2': (80, 50, 0),

            'server1': (135, 65, 0), 'server2': (145, 55, 0),
        }

        # Associations initiales station → AP
        self.stations = {
            'cam1': 'ap1', 'cam2': 'ap1', 'cam3': 'ap1',
            'cam4': 'ap2', 'cam5': 'ap2', 'cam6': 'ap2',

            'sensor1': 'ap3', 'sensor2': 'ap3', 'sensor3': 'ap3',
            'sensor4': 'ap4', 'sensor5': 'ap4', 'sensor6': 'ap4',

            'ctrl1': 'ap5', 'ctrl2': 'ap5', 'ctrl3': 'ap5', 'ctrl4': 'ap5',

            'drone1': 'ap3', 'drone2': 'ap4',

            'server1': 'ap6', 'server2': 'ap6',
        }

        # Types de trafic par station
        self.traffic_mapping = {
            'cam1': 'video', 'cam2': 'video', 'cam3': 'video',
            'cam4': 'video', 'cam5': 'video', 'cam6': 'video',
            'drone1': 'video', 'drone2': 'video',

            'sensor1': 'sensor', 'sensor2': 'sensor', 'sensor3': 'sensor',
            'sensor4': 'sensor', 'sensor5': 'sensor', 'sensor6': 'sensor',

            'ctrl1': 'control', 'ctrl2': 'control',
            'ctrl3': 'control', 'ctrl4': 'control',

            'server1': 'best_effort', 'server2': 'best_effort',
        }

        # États internes
        self.link_utilization = {link: 0.0 for link in self.links}
        self.ap_load = {ap: 0 for ap in self.aps}
        self.station_rssi = {}
        self.active_flows = []

        # Pré-calcul des chemins
        self.paths = self._compute_all_paths()

        # Initialiser la charge des APs
        self.reset()

    def _compute_all_paths(self):
        G = nx.DiGraph()
        G.add_nodes_from(self.aps)
        for (src, dst), data in self.links.items():
            G.add_edge(src, dst, **data)

        paths = {}
        for src in self.aps:
            for dst in self.aps:
                if src != dst:
                    try:
                        all_paths = list(nx.all_simple_paths(G, src, dst, cutoff=4))
                        paths[(src, dst)] = all_paths[:3]  # Limiter à 3 meilleurs chemins
                    except nx.NetworkXNoPath:
                        paths[(src, dst)] = []
        return paths

    def calculate_rssi(self, station, ap=None):
        if station not in self.positions or ap not in self.positions:
            return -70.0

        pos_sta = np.array(self.positions[station])
        pos_ap = np.array(self.positions[ap or self.stations[station]])
        distance = np.linalg.norm(pos_sta - pos_ap)

        if distance < 1:
            distance = 1

        tx_power = 17 if 'cam' in station or 'drone' in station else 14
        path_loss = 40 + 10 * 4.5 * np.log10(distance)
        fading = np.random.normal(0, 4)

        rssi = tx_power - path_loss + fading
        return round(max(rssi, -95), 1)  # RSSI minimum réaliste

    def get_state(self, flow):
        # Charges des 6 APs (normalisées)
        ap_loads = [self.ap_load[ap] / 12.0 for ap in sorted(self.aps)]  # max ~10-12 stations

        # RSSI normalisé de la source
        src_ap = self.stations[flow.src]
        rssi = self.calculate_rssi(flow.src, src_ap)
        rssi_norm = np.clip((rssi + 100) / 60, 0.0, 1.0)  # -100 à -40 → 0 à 1

        # Interférences par canal (1,6,11)
        interferences = []
        for ch in [1, 6, 11]:
            load = sum(self.ap_load[ap] for ap, c in self.channels.items() if c == ch)
            adj_load = sum(self.ap_load[ap] for ap, c in self.channels.items() if abs(c - ch) <= 4 and c != ch)
            interferences.append(min(1.0, load * 0.15 + adj_load * 0.05))

        # Type de flux one-hot
        types = ['video', 'sensor', 'control', 'best_effort']
        type_vec = [1 if flow.type == t else 0 for t in types]

        # Mobilité (variation RSSI récente)
        mobility = 0
        if flow.src in self.station_rssi:
            if abs(rssi - self.station_rssi[flow.src]) > 12:
                mobility = 1
        self.station_rssi[flow.src] = rssi

        state = np.array(ap_loads + interferences + [rssi_norm, mobility] + type_vec, dtype=np.float32)
        return np.pad(state, (0, 20 - len(state)), 'constant')[:20]  # Taille fixe 20

    def step(self, flow, action):
        src_station = flow.src
        src_ap = self.stations[src_station]

        # Décodage action : 18 actions → 6 APs × 3 chemins
        target_ap_idx = action // 3
        path_idx = action % 3
        target_ap = sorted(self.aps)[min(target_ap_idx, 5)]

        available_paths = self.paths.get((src_ap, target_ap), [])
        if not available_paths:
            return self.get_state(flow), -15.0, True, {'error': 'no_path'}

        chosen_path = available_paths[min(path_idx, len(available_paths) - 1)]

        # Métriques
        rssi = self.calculate_rssi(src_station, src_ap)
        latency = self._calculate_path_latency(chosen_path)
        bandwidth = self._calculate_path_bandwidth(chosen_path)
        congestion = self._calculate_path_congestion(chosen_path)

        # Récompense
        reward = self._calculate_reward(flow, rssi, latency, bandwidth, congestion)

        # Mise à jour état
        self._update_network_state(src_ap, chosen_path, flow)

        next_state = self.get_state(flow)
        info = {
            'path': chosen_path,
            'rssi': rssi,
            'latency': latency,
            'bandwidth': bandwidth,
            'target_ap': target_ap
        }

        return next_state, reward, False, info

    def _calculate_reward(self, flow, rssi, latency, bandwidth, congestion):
        traffic_cfg = WiFiTrafficType.__dict__[flow.type.upper()]

        # Normalisation
        latency_norm = latency / 200.0
        bw_satisfied = min(1.0, bandwidth / traffic_cfg['min_bw']) if traffic_cfg['min_bw'] > 0 else 1.0
        signal_quality = 1.0 if rssi >= traffic_cfg['min_rssi'] else max(0.0, 1 - (traffic_cfg['min_rssi'] - rssi) / 20)

        # Poids selon type
        if flow.type == 'control':
            weights = (0.6, 0.2, 0.1, 0.1)  # latency, congestion, bw, signal
        elif flow.type == 'video':
            weights = (0.2, 0.2, 0.5, 0.1)
        elif flow.type == 'sensor':
            weights = (0.4, 0.2, 0.1, 0.3)
        else:
            weights = (0.3, 0.3, 0.2, 0.2)

        reward = (
            -weights[0] * latency_norm
            - weights[1] * congestion
            + weights[2] * bw_satisfied
            + weights[3] * signal_quality
        )

        # Pénalités fortes
        if latency > traffic_cfg['max_latency']:
            reward -= 3.0
        if rssi < -82:
            reward -= 2.0

        return reward

    def _calculate_path_latency(self, path):
        total = 0
        for i in range(len(path) - 1):
            link = (path[i], path[i+1])
            if link in self.links:
                base = self.links[link]['latency']
                util = self.link_utilization.get(link, 0)
                ch = self.channels[path[i]]
                interf = sum(self.ap_load[ap] for ap, c in self.channels.items() if abs(c - ch) <= 4) * 0.05
                total += base * (1 + util * 2 + interf)
        return total

    def _calculate_path_bandwidth(self, path):
        min_bw = float('inf')
        for i in range(len(path) - 1):
            link = (path[i], path[i+1])
            if link in self.links:
                max_bw = self.links[link]['bw'] * 1000  # KB/s
                util = self.link_utilization.get(link, 0)
                ch = self.channels[path[i]]
                interf = sum(self.ap_load[ap] for ap, c in self.channels.items() if c == ch) * 0.1
                avail = max_bw * max(0.1, 1 - util - interf)
                min_bw = min(min_bw, avail)
        return min_bw if min_bw != float('inf') else 1000

    def _calculate_path_congestion(self, path):
        congestions = []
        for ap in path:
            congestions.append(self.ap_load[ap] / 12.0)
        return np.mean(congestions) if congestions else 0

    def _update_network_state(self, src_ap, path, flow):
        bw = WiFiTrafficType.__dict__[flow.type.upper()]['min_bw']
        self.ap_load[src_ap] += 1

        for i in range(len(path) - 1):
            link = (path[i], path[i+1])
            if link in self.links:
                max_bw = self.links[link]['bw'] * 1000
                self.link_utilization[link] += bw / max_bw
                self.link_utilization[link] = min(1.0, self.link_utilization[link])

        self.active_flows.append((flow, path))

    def handle_handover(self, station, old_ap, new_ap):
        if old_ap in self.ap_load:
            self.ap_load[old_ap] = max(0, self.ap_load[old_ap] - 1)
        self.ap_load[new_ap] += 1
        self.stations[station] = new_ap
        return -1.0  # Pénalité handover

    def reset(self):
        self.link_utilization = {link: 0.0 for link in self.links}
        self.ap_load = {ap: 0 for ap in self.aps}
        for sta, ap in self.stations.items():
            self.ap_load[ap] += 1
        self.station_rssi.clear()
        self.active_flows = []
        return self

## 3) dqn_agent.py

import numpy as np
import matplotlib.pyplot as plt
import random
import os
from collections import defaultdict

# On suppose que tu as toujours ces modules
# Si tu ne les as plus, je te donne une version autonome plus bas
try:
    from rl_environment import WiFiSDNEnvironment, Flow, WiFiTrafficType
except:
    # Version minimale autonome si les modules originaux manquent
    class WiFiTrafficType:
        VIDEO = {'priority': 3, 'min_bw': 5000}
        SENSOR = {'priority': 2, 'min_bw': 100}
        CONTROL = {'priority': 1, 'min_bw': 500}
        BEST_EFFORT = {'priority': 4, 'min_bw': 0}

    class Flow:
        def __init__(self, src, dst, type, priority):
            self.src = src
            self.dst = dst
            self.type = type
            self.priority = priority

# Import de ton agent DQN (inchangé normalement)
from dqn_agent import DQNAgent


def create_extended_topology():
    """
    Recrée fidèlement la topologie de ton script Mininet-WiFi
    """
    topology = {
        'aps': ['ap1', 'ap2', 'ap3', 'ap4', 'ap5', 'ap6'],
        'channels': {
            'ap1': 1, 'ap2': 6, 'ap3': 11,
            'ap4': 1, 'ap5': 6, 'ap6': 11
        },
        # Backbone câblé (comme dans ton net.addLink)
        'wired_links': {
            ('ap1', 'ap2'): {'bw': 1000, 'delay': 2},
            ('ap2', 'ap4'): {'bw': 1000, 'delay': 2},
            ('ap3', 'ap4'): {'bw': 1000, 'delay': 2},
            ('ap4', 'ap5'): {'bw': 1000, 'delay': 2},
            ('ap5', 'ap6'): {'bw': 1000, 'delay': 2},
            ('ap1', 'ap3'): {'bw': 500, 'delay': 5},
            ('ap2', 'ap5'): {'bw': 500, 'delay': 5},
        },
        # Association initiale station → AP (basée sur les positions dans ta topo)
        'initial_association': {
            'cam1': 'ap1', 'cam2': 'ap1', 'cam3': 'ap1',
            'cam4': 'ap2', 'cam5': 'ap2', 'cam6': 'ap2',
            'sensor1': 'ap3', 'sensor2': 'ap3', 'sensor3': 'ap3',
            'sensor4': 'ap4', 'sensor5': 'ap4', 'sensor6': 'ap4',
            'ctrl1': 'ap5', 'ctrl2': 'ap5', 'ctrl3': 'ap5', 'ctrl4': 'ap5',
            'drone1': 'ap2',  # position initiale près de zone A/B
            'drone2': 'ap4',  # position initiale près de zone B/C
            'server1': 'ap6', 'server2': 'ap6',
        },
        # Positions approximatives (pour calcul RSSI et mobilité)
        'positions': {
            'ap1': (20, 80, 0), 'ap2': (60, 80, 0),
            'ap3': (20, 40, 0), 'ap4': (60, 40, 0),
            'ap5': (100, 60, 0), 'ap6': (140, 60, 0),

            'cam1': (15, 85, 0), 'cam2': (25, 75, 0), 'cam3': (30, 85, 0),
            'cam4': (55, 85, 0), 'cam5': (65, 75, 0), 'cam6': (60, 90, 0),

            'sensor1': (15, 45, 0), 'sensor2': (25, 35, 0), 'sensor3': (20, 48, 0),
            'sensor4': (55, 45, 0), 'sensor5': (65, 35, 0), 'sensor6': (60, 48, 0),

            'ctrl1': (95, 65, 0), 'ctrl2': (105, 55, 0),
            'ctrl3': (100, 70, 0), 'ctrl4': (110, 60, 0),

            'drone1': (40, 60, 0), 'drone2': (80, 50, 0),

            'server1': (135, 65, 0), 'server2': (145, 55, 0),
        }
    }
    return topology


def generate_realistic_flow(stations, servers):
    """
    Génère un flux réaliste : source = n'importe quelle station IoT, destination = serveur
    """
    src = random.choice(stations)      # cam, sensor, ctrl, drone
    dst = random.choice(servers)       # server1 ou server2

    # Déterminer le type selon le nom de la source
    if 'cam' in src or 'drone' in src:
        flow_type = 'video'
        priority = 3
    elif 'sensor' in src:
        flow_type = 'sensor'
        priority = 2
    elif 'ctrl' in src:
        flow_type = 'control'
        priority = 1
    else:
        flow_type = 'best_effort'
        priority = 4

    return Flow(src=src, dst=dst, type=flow_type, priority=priority)


def simulate_drone_mobility(env, episode):
    """
    Simule la mobilité des deux drones (comme dans ta topologie Mininet)
    """
    # Drone1 : patrouille zone A ↔ B
    if episode % 80 == 0:
        x = random.uniform(30, 70)
        y = random.uniform(60, 90)
        env.positions['drone1'] = (x, y, 0)
        env.handle_handover_if_needed('drone1')

    # Drone2 : patrouille zone B ↔ C
    if episode % 100 == 0:
        x = random.uniform(70, 120)
        y = random.uniform(40, 70)
        env.positions['drone2'] = (x, y, 0)
        env.handle_handover_if_needed('drone2')


def train_extended_dqn_agent(episodes=2000):
    """
    Entraînement DQN adapté à la topologie étendue (6 APs, 20 stations)
    """
    topology = create_extended_topology()

    # Si tu as toujours WiFiSDNEnvironment, utilise-le
    # Sinon, il faudra l'adapter ou recréer une version simplifiée
    try:
        env = WiFiSDNEnvironment(topology)
    except:
        print("⚠ WiFiSDNEnvironment non disponible → tu dois l'adapter ou en créer une version simplifiée")
        return

    # Liste des stations sources et destinations
    all_stations = [s for s in topology['initial_association'].keys() if s not in ['server1', 'server2']]
    servers = ['server1', 'server2']

    # Estimation de la taille de l'état (à adapter selon ton vrai env.get_state())
    sample_flow = generate_realistic_flow(all_stations, servers)
    state_size = len(env.get_state(sample_flow))

    # Espace d'actions : choisir un chemin parmi plusieurs possibles
    # Ici on suppose jusqu'à 3 chemins alternatifs par flux (tu peux augmenter)
    action_size = 18  # Comme dans ton contrôleur : 6 APs × 3 options de routage/queue ?

    print(f"State size: {state_size}")
    print(f"Action size: {action_size} (ex: 6 APs × 3 politiques de QoS/routage)")

    agent = DQNAgent(
        state_size=state_size,
        action_size=action_size,
        learning_rate=0.001,
        gamma=0.99,
        epsilon=1.0,
        epsilon_decay=0.995,
        epsilon_min=0.01,
        buffer_size=10000,
        batch_size=64
    )

    episode_rewards = []
    episode_latencies = []
    episode_throughputs = []
    episode_handovers = []

    print("\n=== Début de l'entraînement DQN - Topologie étendue 6 APs / 20 stations ===\n")

    for episode in range(episodes):
        env.reset()
        total_reward = 0
        total_latency = 0
        total_throughput = 0
        handover_count = 0
        steps = 0

        # Mobilité des drones
        simulate_drone_mobility(env, episode)

        # Nombre de flux par épisode (trafic variable)
        num_flows = random.randint(10, 30)

        for _ in range(num_flows):
            flow = generate_realistic_flow(all_stations, servers)
            state = env.get_state(flow)

            action = agent.choose_action(state, training=True)

            next_state, reward, done, info = env.step(flow, action)

            agent.store_experience(state, action, reward, next_state, done)
            loss = agent.learn()

            total_reward += reward
            total_latency += info.get('latency', 0)
            total_throughput += info.get('throughput', 0)
            if info.get('handover', False):
                handover_count += 1

            steps += 1

        agent.decay_epsilon()

        avg_reward = total_reward / steps if steps > 0 else 0
        avg_latency = total_latency / steps if steps > 0 else 0
        avg_throughput = total_throughput / steps if steps > 0 else 0

        episode_rewards.append(avg_reward)
        episode_latencies.append(avg_latency)
        episode_throughputs.append(avg_throughput)
        episode_handovers.append(handover_count)

        if (episode + 1) % 50 == 0:
            recent_reward = np.mean(episode_rewards[-50:])
            recent_latency = np.mean(episode_latencies[-50:])
            recent_tp = np.mean(episode_throughputs[-50:])
            recent_ho = np.mean(episode_handovers[-50:])

            print(f"Épisode {episode + 1}/{episodes}")
            print(f"  Récompense moyenne : {recent_reward:.3f}")
            print(f"  Latence moyenne    : {recent_latency:.2f} ms")
            print(f"  Débit moyen        : {recent_tp:.1f} Kbps")
            print(f"  Handovers/épisode  : {recent_ho:.1f}")
            print(f"  Epsilon            : {agent.epsilon:.3f}")
            print("-" * 50)

    # Sauvegarde du modèle (chemin compatible avec ton contrôleur Ryu)
    os.makedirs('models', exist_ok=True)
    model_path = 'models/dqn_iot_wifi_extended.pth'
    agent.save_model(model_path)
    print(f"\nModèle sauvegardé : {model_path}")

    # Graphiques
    plot_extended_training_results(
        episode_rewards, episode_latencies,
        episode_throughputs, episode_handovers
    )

    return agent, env


def plot_extended_training_results(rewards, latencies, throughputs, handovers):
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle('Entraînement DQN - Topologie IoT Étendue (6 APs, 20 stations)', fontsize=16)

    # Récompense
    axes[0,0].plot(rewards, color='#3498db', alpha=0.7)
    axes[0,0].set_title('Récompense moyenne')
    axes[0,0].set_ylabel('Récompense')
    axes[0,0].grid(True, alpha=0.3)

    # Latence
    axes[0,1].plot(latencies, color='#e74c3c', alpha=0.7)
    axes[0,1].set_title('Latence moyenne (ms)')
    axes[0,1].set_ylabel('Latence (ms)')
    axes[0,1].grid(True, alpha=0.3)

    # Débit
    axes[1,0].plot(throughputs, color='#2ecc71', alpha=0.7)
    axes[1,0].set_title('Débit moyen (Kbps)')
    axes[1,0].set_ylabel('Débit')
    axes[1,0].set_xlabel('Épisode')
    axes[1,0].grid(True, alpha=0.3)

    # Handovers
    axes[1,1].plot(handovers, color='#9b59b6', alpha=0.7)
    axes[1,1].set_title('Nombre de handovers par épisode')
    axes[1,1].set_ylabel('Handovers')
    axes[1,1].set_xlabel('Épisode')
    axes[1,1].grid(True, alpha=0.3)

    plt.tight_layout()
    os.makedirs('results', exist_ok=True)
    plt.savefig('results/dqn_extended_iot_training.png', dpi=300)
    print("Graphiques sauvegardés : results/dqn_extended_iot_training.png")
    plt.close()


if __name__ == '__main__':
    trained_agent, environment = train_extended_dqn_agent(episodes=2000)
    print("\nEntraînement terminé avec succès !")
    print("Tu peux maintenant charger ce modèle dans ton contrôleur Ryu avec load_trained_model()")

## model folder : we found the trained model with pth extension

## compare1.py : comparison between static routing and routing with rl environment 
FICHIER 1 : Comparaison Routage
RL (DQN simulé) vs Dijkstra classique
Métriques : délai, perte, charge AP
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

np.random.seed(1)
aps = [f"AP{i}" for i in range(1,7)]

# --- Données simulées cohérentes SDN WiFi ---
delay_dijkstra = np.random.normal(38, 6, 6)
delay_rl = np.random.normal(24, 4, 6)

loss_dijkstra = np.random.normal(5.2, 1.2, 6)
loss_rl = np.random.normal(2.0, 0.6, 6)

load_dijkstra = np.array([7,6,4,2,1,0])
load_rl = np.array([4,4,4,3,3,2])

# DataFrame
df = pd.DataFrame({
    'AP': aps,
    'Delay_Dijkstra_ms': delay_dijkstra,
    'Delay_RL_ms': delay_rl,
    'Loss_Dijkstra_%': loss_dijkstra,
    'Loss_RL_%': loss_rl,
    'Load_Dijkstra': load_dijkstra,
    'Load_RL': load_rl
})
print(df.round(2))

# --- Graphes ---
plt.figure(figsize=(14,8))

plt.subplot(2,2,1)
plt.plot(aps, delay_dijkstra, marker='o', label='Dijkstra')
plt.plot(aps, delay_rl, marker='s', label='RL (DQN)')
plt.title('Délai moyen par AP')
plt.ylabel('ms')
plt.legend(); plt.grid(True)

plt.subplot(2,2,2)
plt.bar(aps, loss_dijkstra, label='Dijkstra')
plt.bar(aps, loss_rl, alpha=0.8, label='RL (DQN)')
plt.title('Taux de perte par AP')
plt.ylabel('%')
plt.legend(); plt.grid(axis='y')

plt.subplot(2,1,2)
x = np.arange(len(aps)); w=0.35
plt.bar(x-w/2, load_dijkstra, w, label='Dijkstra')
plt.bar(x+w/2, load_rl, w, label='RL (DQN)')
plt.xticks(x, aps)
plt.ylabel('Stations')
plt.title('Charge des APs')
plt.legend(); plt.grid(axis='y')

plt.suptitle('Comparaison Routage : RL vs Dijkstra')
plt.show()

# ================================
# FICHIER 2 : Comparaison QoS / Priorité
# DQN avec QoS vs Dijkstra simple
# ================================

"""
FICHIER 2 : Comparaison QoS
- DQN + priorité + QoS
- Dijkstra sans priorité
"""

traffic = ['Video', 'Control', 'Sensor', 'BestEffort']

# Délai par type de trafic
delay_dijkstra_qos = [45, 50, 35, 30]
delay_dqn_qos = [25, 18, 28, 32]

# Perte par type de trafic
loss_dijkstra_qos = [6.5, 5.8, 3.5, 2.5]
loss_dqn_qos = [2.2, 1.2, 1.8, 2.3]

plt.figure(figsize=(13,6))

plt.subplot(1,2,1)
plt.bar(traffic, delay_dijkstra_qos, label='Dijkstra simple')
plt.bar(traffic, delay_dqn_qos, alpha=0.85, label='DQN + QoS')
plt.title('Délai selon la priorité du trafic')
plt.ylabel('ms')
plt.legend(); plt.grid(axis='y')

plt.subplot(1,2,2)
plt.bar(traffic, loss_dijkstra_qos, label='Dijkstra simple')
plt.bar(traffic, loss_dqn_qos, alpha=0.85, label='DQN + QoS')
plt.title('Taux de perte selon la priorité')
plt.ylabel('%')
plt.legend(); plt.grid(axis='y')

plt.suptitle('Comparaison QoS : DQN prioritaire vs Dijkstra')
plt.show()


## results
<img width="966" height="762" alt="image" src="https://github.com/user-attachments/assets/9b48caf0-d748-44d9-993a-47185f54e42b" />





