/*
  Telesurveillance Autorail V4
  
  Version SIM7600 sans utilisation MQTT natif
  base V4(ok en 4G hors mqtt)
  Refonte avec librairie TinyGSM revue PhC
  testé avec SIM7600G-H, SIM7600CE-T

  V4-0-4 31/01/2023
  1- Renvoie sur liste restreinte message provenant d'un numéro < 8 chiffres (N° Free)
  
  IDE 1.8.19, AVR boards 1.8.6, PC fixe
	Le croquis utilise 82794 octets (32%), 2097 octets (25%) de mémoire dynamique
  IDE 1.8.19, AVR boards 1.8.6, Raspi
	Le croquis utilise 83166 octets (32%), 2071 octets (25%) de mémoire dynamique

  V4-0-3 installé 13/12/2022 X3944, X4573, X4607, 05/01/2023 X4545, X4554
  08/12/2022
  1- Patch si blocage modem CEREG = 4 (EUTRAN-BAND20 non affectée à free?), reset modem apres cptala defaut
  2- Comptage et remonté dans message vie du nombre de reset modem
  3- Commande SMS consultation nombre de reset modem
  4- Historique changement reseau, recap dans SMS SignalVie et RAZ
  5- Commande SMS lecture Histo reseau

  IDE 1.8.19, AVR boards 1.8.5, PC fixe
	Le croquis utilise 82882 octets (32%), 2097 octets (25%) de mémoire dynamique
  IDE 1.8.19, AVR boards 1.8.5, Raspi
	Le croquis utilise 82910 octets (32%), 2071 octets (25%) de mémoire dynamique

  V4-0-2
  25/11/2022 installé boitier test ancien X4573 réparé(new MEGA2560)
  22/11/2022 installé X4607, X4573 boitier test
  augmentation timeout mise à l'heure initiale
  Relance Majheure sur reception +CNTP: 0
  Revision Majheure, en cas de defaut NTP
  revision Alarme MQTT

  IDE 1.8.19, AVR boards 1.8.5, PC fixe
	Le croquis utilise 81876 octets (32%), 2119 octets (25%) de mémoire dynamique
  IDE 1.8.19, AVR boards 1.8.5, Raspi
	Le croquis utilise 81904 octets (32%), 2093 octets (25%) de mémoire dynamique

  V4-0-1
  31/10/2022 installé X4607 03/11/2022
  IDE 1.8.19, AVR boards 1.8.5, PC fixe
	Le croquis utilise 81742 octets (32%), 2181 octets (26%) de mémoire dynamique
  IDE 1.8.16, AVR boards 1.8.5, Arduino
	Le croquis utilise 81770 octets (32%), 2155 octets (26%) de mémoire dynamique

    
  08/08/2020
  IDE 1.8.19, AVR boards 1.8.4, PC fixe
	Le croquis utilise 77098 octets (30%), 2895 octets (35%) de mémoire dynamique

	IDE 1.8.16 Raspi, AVR boards 1.8.4, Raspi
	Le croquis utilise 77072 octets (30%), 2869 octets (35%) de mémoire dynamique

	Philippe CORBEL
	10/03/2020

	----------------------------------------------
	evolution futur
	----------------------------------------------

  V4-0 08/08/2022 installé X4607
  version 4G sans GPRS/MQTT/ Position opérationnel
  Uniquement SMS pour rester fixe

  V3-12 07/01/2021 pas installé
  externalisation données
  12/06/2021
  course fantaisiste à l'arret, memorisation du dernier course si vitesse = 0
  
  V3-11 29/08/2020 installé boitier de test
  03/09/2020 installé X4573
  11/09/2020 installé X4545,4554,4607,3944
  Calibration par sms possible(installé X4573 le 28/08/2020)
  suppression divisé /10 sur affichage timer alarme bug suite changement de methode timer
  nouveau magique, nouveau param par défaut
  
  V3-105 25/07/2020 installé Picasso
  suppression total CPIN=cpin
  bug affichage compteurmax
  correction gereCadence
  correction connexion mqtt, evite boucle bloquante

  V3-104 08/07/2020
  bug tracker timerlent/timerrapide

  V3-103 10/03/2020 installé 14/05/2020 X3944, X4554
  !!!!! Version carte SIM sans codePIN !!!!!
  1 - reprise de V2-22
  2 - correction bug RAZ Fausses alarmes si Alarme en cours
  3 - Installation de la localisation dynamique par GPRS
      Nouveau magic
      Modification Librairie FONA 1.3.106
      nouvelle commandes Tracker ON/OFF, GPRSDATA,
      MQTTDATA seul l'adresse serveur est modifiable utiliser MQTTserveur
  4 - dans traite_sms(), Suppression du sms au début avant traitement,
      si traitement long, evite de traiter 2 fois le meme sms
  5 - enregistrement en EEPROM séparement de Coefftension(independant de magic)
  6 - suppression code PIN SIM et verif cnx reseau


	V2-22 19/11/2019 pas encore installé
  1 - modification sms MAJHEURE idem PNV2-1

  V2-21 24/06/2019 installé 02/07/2019 X4545,4573,4607,3944, 22/07/2019 X4554
	1 - Ajout date et heure sur tous les messages
	2 - suppression resetsim dans majheure
	3 - Creation message FALARME retourne etat des fausses alarmes meme sans Alarme en cours

	V2-20 25/10/2018 installé testé sur boitier SPARE
	installé sur X4545 et 4554 le 27/09/2018, sur X4573, X3944 et X4607 le 02/11/2018
	1 - inclure valeur compteur temps (timecompte) dans message Alarme intrusion
	2 - lors de l'armement Alarme, inclure dans message retour info Capteur 0/1 si au moins un capteur = 0
	3 - suppression demarrage en boucle
	4 - suppression resetsoft si pas de réseau remplacé par ResetSim8OO
	5 - nouveau parametre par defaut jour 3-60, nuit 2-90

	V2-19 11/05/2018 installé sur boitier SPARE
	1 - Bug sur PC Portable pas de remise à jour ou mise à jour incomplete EEPROM si nouveau magic
			Augmentation delay apres lecture EEPROM 500ms
	2 - ajout commande MAJHEURE, effectue "reset soft" SIM800 et lance mise a l'heure

	13/03/2018 modif sur X4573 sans evolution version ligne marquée 13/03/2018
	securisation lancement GPS verification en meme temps que SIM dans la boucle

	a faire verif SIM plusieur fois avant envoyer de nouveau cod SIM?

	V2-18 26/01/2018 installé sur X3944 apres modif hard

	V2-18 19/12/2017 installé le 21/12/2017 sur X4545, X4573, X4554, X4607
	1- Demarrage capteurs en boucle pour essayer de garantir démarrage à froid
	2- Correction bug date dans log

	V2-17 11/12/2017 chargé sur X4573
	1- Correction bug blocagealarme voir descriptif (Bug 20171207.txt)
	2- forcer remise à l'heure à chaque reception PSUTTZ sans verif année
	3- bug a corriger sur reception message Silence OFF commande inverseée avec ON
	4- ajouter commande pour recuperer IEMI

	V2-16 09/11/2017 installé X4545, X4554, X4573
	1-correction bug etat batterie en %
	2-ajout hysteresis et tempo retour sur detection alarme tension batterie
	Alarme 11.62V/20%, retour 12.42V/80%

	V2-15 installé 26/10/2017 sur X4545, X4554, X4573
	ajout info % charge batterie
	ref : http://www.regenebatt.com/batterie-world/diagnostique-batterie/etat-de-charge-avec-un-multimetre.html

	V2-14 installé sur boitier du
	X4554 le 23/09/2017
	X4545 le 03/10/2017
	X4573 le 04/10/2017
  1- Allumage voyants de controle Alarme dans la boucle ne permet pas de voir les alarmes courtes,
		 bien quelle soit comptabilisé correctement
		 ajout dans les IRQ allumage du voyant Led_PIRX
  2- bloquage au demarrage si batterie lipo faible, reste bloqué sur while(1)
		 remplacé par softreset tente de redemarrer jusqu'a ce que la batterie soit rechargée
		 reorganisation verif SIM en une routine appelée au lancement
		 et ajouté dans boucle acquisition verif etat SIM et reset si pb
	3- memorisation log en EEPROM que si different de l'etat courant Axxxx Dxxxx
	4- ajout tempo apres lecture et ecriture EEPROM
		 ajout impression des data EEPROM au lancement
	5- deplacement du code allumage capteur apres initialisation GSM
	6- lancement sirene et SMS directement sans attendre prochaine boucle acquisition
	7- nouveau num magic 12349

	V2-122 installé sur autorail X4545 et X4573 26/07/2017
	Bug:
	1-A/D4545/4573 ne fonctionne pas compilateur PCportable 44638/1647?44680/1651 octets PCfixe,
	2-numero magic ??

	memorisé coeff tension batterie propre a chaque carte dans EEPROM
	procedure de calibration
	moyennage de la tension mesurée, entree analogique en variable
	ajout changement PARAM jour/nuit
	variable config.timecomptemax devient TmCptMax
	variable config.Nmax devient Nmax
	elles prennent la valeur Jour/Nuit selon plage horaire
	entraine modif num magic
	et position EEPROM pour record(log) recordn=75

	V2-12  pas encore installé sur autorail

	ajout fonction lancement sirene sur demande SIRENE
	bug corrigé
	si capteur actif/inactif modifié, les interruptions ne sont pas mise à jour
	Affichage tension batterie decimale erronées
	PARAM non sauv en EEPROM apres changement

	V2-11 installée sur X4545 V2 26/06/2017
	correction coeff tension batterie
	correction message horaire Fausses Alarmes
	mettre Faussealarme = 1000 si detection coupure permanente

	installé le 14/06/2017 sur X4573

	Telesurveillance Autorail V2 X4573
	sur base de V1-14

	Gestion independante des 4 capteurs PIR RX et TX
	Armement independant des 4 capteurs PIR sur demande
	Log enregistrée en EEPROM
	rebond reduit de 100 à 50 (filtrage hard)
	fausses alarmes >3 en 2min en parametres et EEPROM
	suppression watchdog, conservé pour Reset , modification du Soft reset

	EEPROM
	adrr=confign=0	adresse EEPROM structure config. enregistre tous les parametres en EEPROM
	adrr=recordn=50 adresse EEPROM structure Log

	Librairie TimeAlarms.h modifiée
	#define dtNBR_ALARMS 12
  ne pas utiliser enable, remplacé par write

*/
#include <Arduino.h>

const String ver = "V4-0-4";
int Magique = 16;

#define TINY_GSM_MODEM_SIM7600

#include "defs.h"
#include <EEPROM.h>							// variable en EEPROM
#include <EEPROMAnything.h>			// variable en EEPROM
#include <Time.h>								// gestion Heure
#include <TimeAlarms.h>					// gestion des Alarmes
#include <avr/wdt.h>						// watchdog uniquement pour Reset
#include <ArduinoJson.h>
#include <credentials_tpcf.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
/*  FONA_RX       2	==>     mega14 TX3
		FONA_TX       3	==>     mega15 RX3
		Entree Tension Batterie A5	*/
#define PowerKey      12        // SIM7600 Power management
#define HardReset     10        // Reset Hard pas utilisé
#define Ip_Analogique 5					// entree analogique mesure tension	V2-122
#define FONA_RST      4					// Reset SIM800
#define FONA_RI       6					// Ring SIM800
#define LED_PIN				13				// voyant 13
#define S_Son         40				// Sortie commande Sirène
#define Ip_PIR1				18				// Entrée Alarme 1 motrice 	droite
#define Ip_PIR2				19				// Entrée Alarme 2 motrice 	gauche
#define Ip_PIR3				20				// Entrée Alarme 3 remorque droite
#define Ip_PIR4				21				// Entrée Alarme 4 remorque gauche

// numero a definir pour mega, Interrupt 2, 3,
#define led						38				// Sortie Led Verte clignotante
#define led_PIR1			22				// Sortie Voyant indic PIR1
#define led_PIR2			26				// Sortie Voyant indic PIR2
#define led_PIR3			30				// Sortie Voyant indic PIR3
#define led_PIR4			34				// Sortie Voyant indic PIR4
#define Op_PIR1				24				// Sortie Alimentation PIR1
#define Op_PIR2				28				// Sortie Alimentation PIR2
#define Op_PIR3				32				// Sortie Alimentation PIR1
#define Op_PIR4				36				// Sortie Alimentation PIR2

#define NTPServer "pool.ntp.org"

/************ Global State (you don't need to change this!) ******************/
// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

/*
	Entrée A5 Mesure Tension Batterie 24V R33k/3.3k => 30V/2.73V
	Vref externe 2.889V R5.1k entre 3.3V et Vref
*/

String fl = "\n";								//	saut de ligne SMS
String Id ;											// 	Id du PN sera lu dans EEPROM
String Sbidon = "";             // variable tempo
// #define Serial Serial
#define SerialAT Serial3
// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// #define TINY_GSM_USE_GPRS true
// #define TINY_GSM_USE_WIFI false
// set GSM PIN, if any
#define GSM_PIN ""

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient  mqtt(client);
PhonebookEntry Phone;

// #define Analyse_boucle // pour analyse temps de boucle

String  message;											//	Texte des SMS envoyé en reponse

char    fonaInBuffer[64];      				//	for notifications from the modem

bool FlagPIR 								= false;	// detection PIR

bool FlagAlarmeTension 			= false;	//	Alarme tension Batterie
bool FlagLastAlarmeTension	= false;

bool FlagAlarmeIntrusion		= false;	//	Alarme Intrusion detectée
bool FlagLastAlarmeIntrusion = false;

bool FlagMessageLocal   = false;
bool FlagAlarmeGprs     = false; // Alarme confirmée
bool AlarmeGprs         = false; // detection alarme
bool FlagLastAlarmeGprs = false;
bool FlagAlarmeMQTT     = false;
bool AlarmeMQTT         = false;
bool FlagLastAlarmeMQTT = false;
bool FlagAlarmeGps      = false;
bool AlarmeGps          = false;
bool FlagLastAlarmeGps  = false;
bool lancement          = false;    // passe a true apres lancement

bool FlagReset 					= false;	  //	Reset demandé=True
long TensionBatterie;									//	Tension Batterie solaire 12V

float   lat;													// latitude
float   lon;													// longitude
float   speed = 0.0;									// vitesse
bool    flagtestspeed = false;
float   heading = 0.0;								// cap

#ifdef Analyse_boucle
uint32_t tboucled = 0; // pour mesurer temps de boucle avec lecture GPS
uint32_t tbouclemoyen=0;
int cptboucle = 0; // compteur boucle
#endif

bool    FirstSonn = false;					// Premier appel sonnerie
bool    SonnMax   = false;					// temps de sonnerie maxi atteint
byte 		CptTest   = 12;							// décompteur en mode test si=0 retour tempo normale
// V2-122
bool FlagCalibration = false;         // Calibration Tension en cours
int CoeffTensionDefaut = 3100;        // Coefficient par defaut
int CoeffTension = CoeffTensionDefaut;// Coefficient calibration Tension relu en EEPROM
int 		TmCptMax 	= 0;							// Temps de la boucle fausses alarme
int     Nmax			= 0;							// Nombre de fausses alarmes avant alarme
int NbrResetModem = 0;              // Nombre de fois reset modem, remise à 0 signal vie
int Histo_Reseau[5]={0,0,0,0,0};    // Historique Reseau cumul chaque heure
// V2-122

struct  config_t 										// Structure configuration sauvée en EEPROM
{
  int 		magic		;									// num magique
  long 		Ala_Vie ;									// Heure message Vie, 8h matin en seconde = 8*60*60
  bool    Intru   ;									// Alarme Intrusion active
  bool    Silence ;									// Mode Silencieux = true false par defaut
  bool    Pos_PN 	;									// envoie un SMS au premier lancement apres 30s
  bool    Pos_Pn_PB[10];						// numero du Phone Book (1-9) à qui envoyer 0/1 0 par defaut
  int 		Dsonn 	;									// Durée Sonnerie
  int 		DsonnMax;									// Durée Max Sonnerie
  int 		Dsonnrepos;								// Durée repos Sonnerie
  int 		Jour_TmCptMax;	 					// Jour Temps de la boucle fausses alarme s
  int 		Jour_Nmax ;				        // Jour Nombre de fausses alarmes avant alarme V2-122
  char 		Idchar[11];								// Id
  byte    PirActif[4];							// Capteur PIR actif
  // V2-122
  bool    IntruAuto;								// pas utilisé Mode Alarme Intrusion automatique entre Hsoir et Hmatin
  long 		IntruFin;									// Heure arret Alarme Intru Matin parametre jour/nuit
  long 		IntruDebut;								// Heure debut Alarme Intru Soir
  int 		Nuit_TmCptMax;						// Nuit Temps de la boucle fausses alarme s
  int 		Nuit_Nmax ;				        // Nuit Nombre de fausses alarmes avant alarme
  // V2-122
  bool    tracker;                  // tracker ON/OFF
  int     trapide;                  // timer send data rapide (roulage)
  int     tlent;                    // timer send data lent (arret)
  int     vtransition;              // vitesse transition arret/roulage
  char    apn[11];                  // APN
  char    gprsUser[11];             // user for APN
  char    gprsPass[11];             // pass for APN
  char    mqttServer[26];           // Serveur MQTT
  char    mqttUserName[11];         // MQTT User
  char    mqttPass[16];             // MQTT pass
  char    writeTopic[16];           // channel Id
  uint16_t mqttPort;                // Port serveur MQTT
  byte    cptAla;                   // Compteur alarmes Tracker avant declenchement
  int     hete;                     // decalage Heure été UTC
  int     hhiver;                   // decalage Heure hiver UTC
} config;

byte EEPROM_adresse[3] = {0, 20, 170}; // Adresse EEPROM 0:coefftension,1:log,2:config

typedef struct        // declaration structure  pour les log
{
  char 		dt[10];     //	DateTime 0610-1702 9+1
  char 		Act[2];     //	Action A/D/S/s 1+1
  char 		Name[15];   //	14 car
} champ;
champ record[5];

byte Accu = 0;                    // accumulateur/amortisseur vitesse

bool FlagTempoIntru 	= false;		// memorise config.Intru au demarrage

// V1-12 - V1-13
volatile int CptAlarme1	 = 0;				//	compteur alarme avant filtrage M1
volatile int CptAlarme2	 = 0;				//	compteur alarme avant filtrage M2
volatile int CptAlarme3	 = 0;				//	compteur alarme avant filtrage R1
volatile int CptAlarme4	 = 0;				//	compteur alarme avant filtrage R2
int 			 FausseAlarme1 = 0;				//	compteur fausse alarme
int 			 FausseAlarme2 = 0;				//	compteur fausse alarme
int 			 FausseAlarme3 = 0;				//	compteur fausse alarme
int 			 FausseAlarme4 = 0;				//	compteur fausse alarme
volatile unsigned long rebond1 = 0;	//	antirebond IRQ
volatile unsigned long rebond2 = 0;
volatile unsigned long rebond3 = 0;
volatile unsigned long rebond4 = 0;
int timecompte 	  = 0;     // comptage nbr passage dans loop compteur temps fausses alarmes
int timecomptememo = 0;		 // Memorisation timecompte si alarme detecté V2-20
// int timecomptemax = 1200;	 // Temps de la boucle fausses alarme 1200 = 2mn
// int Nmax = 3;             //  Nombre de fausses alarmes avant alarme
// V1-12 - V1-13
int N_Y, N_M, N_D, N_H, N_m, N_S; // variable Date/Time temporaire
uint32_t lastReconnectMQTTAttempt = 0;
uint32_t lastReconnectGPRSAttempt = 0;

/* Identification des Alarmes*/
AlarmId FirstMessage;		// 0 tempo lancement premier message et activation Alarme au demarrage
AlarmId loopPrincipale;	// 1 boucle principlae
AlarmId Svie;						// 2 tempo Signal de Vie
AlarmId MajH;						// 3 tempo mise à l'heure régulière
AlarmId TSonn;					// 4 tempo durée de la sonnerie
AlarmId TSonnMax;				// 5 tempo maximum de sonnerie
AlarmId TSonnRepos;			// 6 tempo repos apres maxi
AlarmId HIntruF;				// 7 Heure Fin Matin parametre  //V2-122
AlarmId HIntruD;				// 8 Heure Debut Soir parametre //V2-122
AlarmId Analyse;				// 9 boucle analyse alarme
AlarmId Send;           //   timer send data localisation

//---------------------------------------------------------------------------
void IRQ_PIR1() {				// Detection PIR1
  if (config.PirActif[0]) {					// si capteur actif
    if (millis() - rebond1 > 50) {	// Antirebond
      CptAlarme1 ++;
      digitalWrite(led_PIR1, HIGH);  // V2-14 allume led locale temoin alarme
      rebond1 = millis();
    }
  }
}

void IRQ_PIR2() {				// Detection PIR2
  if (config.PirActif[1]) {					// si capteur actif
    if (millis() - rebond2 > 50) {	// Antirebond
      CptAlarme2 ++;
      digitalWrite(led_PIR2, HIGH);  // V2-14 allume led locale temoin alarme
      rebond2 = millis();
    }
  }
}

void IRQ_PIR3() {				// Detection PIR3
  if (config.PirActif[2]) {					// si capteur actif
    if (millis() - rebond3 > 50) {	// Antirebond
      CptAlarme3 ++;
      digitalWrite(led_PIR3, HIGH);  // V2-14 allume led locale temoin alarme
      rebond3 = millis();
    }
  }
}

void IRQ_PIR4() {				// Detection PIR4
  if (config.PirActif[3]) {					// si capteur actif
    if (millis() - rebond4 > 50) {	// Antirebond
      CptAlarme4 ++;
      digitalWrite(led_PIR4, HIGH);  // V2-14 allume led locale temoin alarme
      rebond4 = millis();
    }
  }
}
//---------------------------------------------------------------------------
void MajHeure(bool force = false);
//---------------------------------------------------------------------------
void setup() {
  message.reserve(255);			 // texte des SMS
  Serial.begin(115200);			 //	Liaison Serie PC ex 115200
  while (!Serial);
  Serial.println(__FILE__);
  Serial.print(F("Version Soft : ")), Serial.println(ver);
  pinMode(LED_PIN, OUTPUT);

  // Activation SIM7600
  pinMode(PowerKey, OUTPUT);
  digitalWrite(PowerKey,LOW);
  delay(500);
  digitalWrite(PowerKey,HIGH);

  // Set GSM module baud rate
  // pour nouvelle carte avant premiere utilisation faire AT+IPREX=38400
  // certaine cde comme ATI modeminfo ne fonctionne pas à 115200(val defaut usine),
  // Ok à 57600, on garde une marge un cran en dessous = 38400 
  SerialAT.begin(38400);
  Serial.println(F("Initializing modem..."));
  unsigned long debut = millis();
  while(!modem.init()){
    delay(1000);
    if(millis() - debut > 60000) break;
  }
  // modem.restart();
  Serial.print(F("Modem Info: "));
  Serial.println(modem.getModemInfo());


  /* Lecture configuration en EEPROM	 */
  EEPROM_readAnything(EEPROM_adresse[0], CoeffTension);
  Alarm.delay(500);	//	V2-19
  EEPROM_readAnything(EEPROM_adresse[1], record);
  Alarm.delay(500);
  EEPROM_readAnything(EEPROM_adresse[2], config);

  if (config.magic != Magique) {											//V2-14
    /* verification numero magique si different
        erreur lecture EEPROM ou carte vierge
    		on charge les valeurs par défaut */
    Serial.println(F("Nouvelle carte vierge !"));
    config.magic 				 = Magique;										//V2-14
    config.Ala_Vie 			 = 25200;	// 25200 = 7h00
    config.Intru 				 = true;
    config.Silence			 = false;
    config.Pos_PN				 = true;
    config.Dsonn				 = 60;
    config.DsonnMax			 = 90;
    config.Dsonnrepos    = 120;
    config.Jour_TmCptMax = 90;// V2-20 60// 30s // V2-14
    config.Jour_Nmax		 = 2;						// V2-14
    config.PirActif[0]   = 1; // tous les capteurs PIR sont actif par défaut
    config.PirActif[1]   = 1;
    config.PirActif[2]   = 1;
    config.PirActif[3]   = 1;
    String temp 				 =	"TPCF_X4500";
    temp.toCharArray(config.Idchar, 11);
    // V2-122
    config.IntruAuto		 = true;			// pas utilisé
    config.IntruFin		 	 = 21600; 		// 06h00 21600
    config.IntruDebut		 = 75600; 		// 21h00 75600
    config.Nuit_TmCptMax = 90;       // V2-20 90//60s V2-14
    config.Nuit_Nmax		 = 2;        // V2-14
    // V2-122
    config.tracker          = false;
    config.trapide          = 15;      // secondes
    config.tlent            = 15;// * 60; // secondes
    config.vtransition      = 2;       // kmh
    String tempapn          = "free";//"sl2sfr";//"free";
    String tempUser         = "";
    String tempPass         = "";
    config.mqttPort         = tempmqttPort;
    config.cptAla           = 10; // 11*Acquisition time
    config.hete             = 2; // heure
    config.hhiver           = 1; // heure
    tempapn.toCharArray(config.apn, (tempapn.length() + 1));
    tempUser.toCharArray(config.gprsUser, (tempUser.length() + 1));
    tempPass.toCharArray(config.gprsPass, (tempPass.length() + 1));
    tempServer.toCharArray(config.mqttServer, (tempServer.length() + 1));
    tempmqttUserName.toCharArray(config.mqttUserName, (tempmqttUserName.length() + 1));
    tempmqttPass.toCharArray(config.mqttPass, (tempmqttPass.length() + 1));
    temptopic.toCharArray(config.writeTopic, (temptopic.length() + 1));
    for (int i = 0; i < 10; i++) {// initialise liste PhoneBook liste restreinte
      config.Pos_Pn_PB[i] = 0;
    }
    config.Pos_Pn_PB[1] = 1;	// le premier numero du PB par defaut

    int longueur = EEPROM_writeAnything(EEPROM_adresse[2], config);	// ecriture des valeurs par defaut
    Alarm.delay(350);					//V2-14
    Serial.print(F("longEEPROM1=")), Serial.println(longueur); //long=170

    // valeur par defaut des record (log)
    for (int i = 0; i < 5 ; i++) {
      temp = "";
      temp.toCharArray(record[i].dt, 10);
      temp.toCharArray(record[i].Act, 2);
      temp.toCharArray(record[i].Name, 15);
    }
    longueur = EEPROM_writeAnything(EEPROM_adresse[1], record);	// ecriture des valeurs par defaut
    Alarm.delay(250);						//V2-14
    Serial.print(F("longEEPROM2=")), Serial.println(longueur); //long=135
  }
  PrintEEPROM();

  Id  = String(config.Idchar);
  Id += fl;

  analogReference(EXTERNAL);// reference Analog 3.3V au travers de 5.1K  Vref=2.85V, 1023=31.32V

  pinMode(Ip_PIR1, INPUT_PULLUP);					// Entrée Detecteur PIR1
  pinMode(Ip_PIR2, INPUT_PULLUP);					// Entrée Detecteur PIR2
  pinMode(Ip_PIR3, INPUT_PULLUP);					// Entrée Detecteur PIR3
  pinMode(Ip_PIR4, INPUT_PULLUP);					// Entrée Detecteur PIR4
  pinMode(led_PIR1, OUTPUT);							// Sortie voyant
  pinMode(led_PIR2, OUTPUT);							// Sortie voyant
  pinMode(led_PIR3, OUTPUT);							// Sortie voyant
  pinMode(led_PIR4, OUTPUT);							// Sortie voyant
  pinMode(led, OUTPUT);										// Sortie voyant activité
  pinMode(S_Son, OUTPUT);									// Sortie commande Sonnerie
  pinMode(Op_PIR1, OUTPUT);								// Sortie alim PIR1
  pinMode(Op_PIR2, OUTPUT);								// Sortie alim PIR2
  pinMode(Op_PIR3, OUTPUT);								// Sortie alim PIR3
  pinMode(Op_PIR4, OUTPUT);								// Sortie alim PIR4
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(S_Son, LOW);								// Sortie Sonnerie OFF
  digitalWrite(led, LOW);									// Voyant activité
  digitalWrite(led_PIR1, LOW);						// Voyant PIR1
  digitalWrite(led_PIR2, LOW);						// Voyant PIR2
  digitalWrite(led_PIR3, LOW);						// Voyant PIR3
  digitalWrite(led_PIR4, LOW);						// Voyant PIR4
  digitalWrite(LED_PIN, LOW);									// Eteindre voyant 13
  digitalWrite(Op_PIR1, LOW);							// Alimentation Capteur à 0
  digitalWrite(Op_PIR2, LOW);
  digitalWrite(Op_PIR3, LOW);
  digitalWrite(Op_PIR4, LOW);

  Serial.println(F("Lancement Application "));
  Serial.print(("Waiting for network..."));
  if (!modem.waitForNetwork()) {
    Serial.println(F(" fail"));
    delay(100);
    // return;
  } else {
    Serial.println(F(" success"));
  }
  if (modem.isNetworkConnected()) { Serial.println(F("Network connected")); }

  // GPRS connection parameters are usually set after network registration
  Serial.print(F("Connecting to "));
  Serial.print(config.apn);
  if (!modem.gprsConnect(config.apn, config.gprsUser, config.gprsPass)) {
    Serial.println(F(" fail"));
    delay(10000);
    // return;
  }
  Serial.println(F(" success"));
  if (modem.isGprsConnected()) { Serial.println(F("GPRS connected")); }

  // code deplacé était avant initialisation GSM version precedente V2-14
  if (config.Intru) {									// si Alarme Intru active
    FlagTempoIntru = config.Intru;		// on memorise
    config.Intru = false;							// on desactive jusqu'a tempo demarrage 1mn
    AllumeCapteur();									// allumage des capteurs selon parametres
  }


  // VerifSIM();							// V2-14 verification si SIM et deverrouillage

  byte n;
  byte cpt = 0;
  do {												// boucle tant que reseau pas connecté
    Alarm.delay(100);
    n = modem.getRegistrationStatus();
    cpt ++;
    if (cpt > 2) break;				// sortie si 2 tentatives demarrage sans reseau
  } while (!(n == 1 || n == 5));
  Serial.print(F("Network status "));
  Serial.print(n);
  Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  flushSerial();
  // Demande Operateur connecté
  Serial.print(F("Operateur :")), Serial.println(modem.getOperator());

  // Synchro heure réseau du modem
  Serial.println(F("Synchro Heure réseau "));
  if(SyncHeureModem(config.hete*4, true)){ // heure été par defaut, first time
    Serial.println(F("OK"));
  } else {Serial.println(F("KO"));}
  Serial.println(modem.getGSMDateTime(TinyGSMDateTimeFormat(0)));

  message = "";
  read_RSSI();									 // Niveau reseau
  Serial.println(message);
  message = "";

  // Allumage du GPS
  Serial.println(F("turn ON GPS"));
  modem.enableGPS();
  
// MQTT Broker setup
  mqtt.setServer(config.mqttServer, config.mqttPort);//1883
  // mqtt.setCallback(mqttCallback); // a creer quand besoin

  timesstatus();								// Etat synchronisation Heure Sys
  decodeGPS();									// Etat GPS
  MajHeure();										// Mise à jour Date et Heure systeme depuis réseau

  /* parametrage des Alarmes */

  FirstMessage = Alarm.timerOnce(30, OnceOnly); // appeler une fois apres 30 secondes type=0

  loopPrincipale = Alarm.timerRepeat(15, Acquisition); // boucle principale 15s
  Alarm.enable(loopPrincipale);

  MajH = Alarm.timerRepeat(3600, MajHeure);						// toute les heures  type=1
  Alarm.enable(MajH);

  TSonn = Alarm.timerRepeat(config.Dsonn, ArretSonnerie);		// tempo durée de la sonnerie
  Alarm.disable(TSonn);

  TSonnMax = Alarm.timerRepeat(config.DsonnMax, SonnerieMax); // tempo maximum de sonnerie
  Alarm.disable(TSonnMax);

  TSonnRepos = Alarm.timerRepeat(config.Dsonnrepos, ResetSonnerie); // tempo repos apres maxi
  Alarm.disable(TSonnRepos);

  Svie = Alarm.alarmRepeat(config.Ala_Vie, SignalVie); // chaque jour type=3
  //Serial.print(F("Alarme vie =")),Serial.println(Alarm.read(Svie));
  Alarm.enable(Svie);
  //V2-122
  HIntruF = Alarm.alarmRepeat(config.IntruFin, IntruF);
  HIntruD = Alarm.alarmRepeat(config.IntruDebut , IntruD);
  Alarm.enable(HIntruD);
  Alarm.enable(HIntruF);
  //V2-122
  Analyse = Alarm.timerRepeat(1, AnalyseAlarme);
  Alarm.enable(Analyse);
  Send = Alarm.timerRepeat(config.tlent, senddata); // send data
  Alarm.disable(Send);

  Serial.print(F("FreeRAM = ")), Serial.println(freeRam());

  modem.setNetworkMode(2); // force Network Mode Auto

}	//fin setup

//---------------------------------------------------------------------------
void loop() {
  #ifdef Analyse_boucle
  tboucled = millis();
  #endif
  recvOneChar();// liaison serie en local

  if (FlagMessageLocal) {
    FlagMessageLocal = false;
    traite_sms(99);
  }
  
  if (rebond1 > millis()) rebond1 = millis();
  if (rebond2 > millis()) rebond2 = millis();
  if (rebond3 > millis()) rebond3 = millis();
  if (rebond4 > millis()) rebond4 = millis();

  // Traitement des messages non sollicté en provenance modem
  // Attente donnée en provenance SIM800/SIM7600
  String bufferrcpt;
  char* bufPtr ;//= fonaInBuffer;		//handy buffer pointer
  if (SerialAT.available()>0) {   //any data available from the modem?
    byte slot = 0;            		//this will be the slot number of the SMS
    // int charCount = 0;
    //Read the notification into fonaInBuffer
    do  {
      *bufPtr = SerialAT.read();
      bufferrcpt += *bufPtr;
      Serial.write(*bufPtr);
      // Alarm.delay(1);
    } while ((*bufPtr++ != '\n') && (SerialAT.available()) );// && (++charCount < (sizeof(fonaInBuffer) - 1)));
    //Add a terminal NULL to the notification string
    *bufPtr = 0;
    // if (charCount > 1) {
      // Serial.print(F("Buffer ="));
      // Serial.println(bufferrcpt);
    // }
    // Si appel entrant on raccroche
    if ((bufferrcpt.indexOf(F("RING"))) == 0) {	// RING, Ca sonne
      // Serial.println(F("Ca sonne!!!!"));
      modem.callHangup();											// on raccroche
    }
    if ((bufferrcpt.indexOf(F("PSUTTZ"))) >= 0 ) { // V2-17 rattrapage si erreur mise à la date
      Serial.println(F("Relance mise à l'heure !"));
      //FlagReset = true;	// on force redemarrage pour prendre la bonne date/time
      MajHeure();
    }
    if ((bufferrcpt.indexOf(F("+CNTP: 0"))) >= 0 ) { // V4-0-2 si reception tardive relance majheure
      Serial.println(F("Relance mise à l'heure !"));
      MajHeure();
    }
    if (bufferrcpt.indexOf(F("CMTI")) >= 0 ) { 
      int p = bufferrcpt.indexOf(",");
      slot = bufferrcpt.substring(p+1,bufferrcpt.length()).toInt();
      // Serial.print(F(", SMS en reception:")),Serial.println(slot);
      traite_sms(slot);
    }
    // Scan the notification string for an SMS received notification.
    // If it's an SMS message, we'll get the slot number in 'slot'
    // if (1 == sscanf(fonaInBuffer, "+CMTI: \"SM\",%d", &slot)) {
    //   traite_sms(slot);
    // }
  }

  if(!config.Intru && config.tracker && lancement){  
    // Make sure we're still registered on the network
    // ne fonctionne pas si modem bloqué CGREG = 4
    // if (!modem.isNetworkConnected()) { // CGREG GPRS network registration status
    //   Serial.println(F("Network disconnected"));
    //   if (!modem.waitForNetwork(180000L, true)) {
    //     Serial.println(F(" fail"));
    //     // delay(10000);
    //     // return;
    //   }
    //   if (modem.isNetworkConnected()) {
    //     Serial.println(F("Network re-connected"));
    //   }
    // }


    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) { // NETOPEN? Start TCPIP service
      Serial.println(F("GPRS disconnected!"));
      Serial.print(F("Connecting to "));
      Serial.print(config.apn);
      if (!modem.gprsConnect(config.apn, config.gprsUser, config.gprsPass)) {
        Serial.println(F(" fail"));
        if (millis() - lastReconnectGPRSAttempt > 10000L){
          lastReconnectGPRSAttempt = millis();
          AlarmeGprs = true;
        }
        // delay(10000);
        // return
      } else {lastReconnectGPRSAttempt = 0;}
      if (modem.isGprsConnected()) {
        Serial.println(F(" GPRS reconnected"));
        AlarmeGprs = false;
      }
    }
    if (!mqtt.connected()) {      
      // Reconnect every 10 seconds
      if (millis() - lastReconnectMQTTAttempt > 10000L) {
        Serial.println(F("=== MQTT NOT CONNECTED ==="));
        lastReconnectMQTTAttempt = millis();
        AlarmeMQTT = true;
        if (mqttConnect()) {
          lastReconnectMQTTAttempt = millis();
          AlarmeMQTT = false;
          }
      }
      // delay(100);
      // return;
    }
    mqtt.loop();
  }

  gereVoyant();
  Alarm.delay(0);
  #ifdef Analyse_boucle
  cptboucle +=1;
  tbouclemoyen += ((millis()-tboucled)/cptboucle);
  #endif
}	//fin loop
//---------------------------------------------------------------------------
void AnalyseAlarme() {
  static bool timerlance = false;						//	activation timer alarme 1mn
  if (config.Intru && (CptAlarme1 > 0 || CptAlarme2 > 0 || CptAlarme3 > 0 || CptAlarme4 > 0 )) {
    if (map(moyenneAnalogique(), 0, 1023, 0, CoeffTension) > 1150) { // prise en compte seulement si Vbatt OK, fausse alarme si Vbatt coupée	V2-122
      if (!timerlance)timerlance = true;				// on lance le timer si pas deja fait
      timecompte ++;	// on incremente le compteur temps Alarme

      if ((CptAlarme1 > Nmax || CptAlarme2 > Nmax || CptAlarme3 > Nmax || CptAlarme4 > Nmax) && timecompte < TmCptMax) {		// Alarme validée V2-122
        FlagPIR = true;
        timecomptememo = timecompte;	// V2-20
      }

      if (timecompte > TmCptMax || FlagPIR) { 	// remise à 0 du comptage apres delai ou alarme detectée		V2-122
        timerlance = false; 									// on arrete le timer
        timecompte = 0;
        FausseAlarme1  += CptAlarme1;
        FausseAlarme2  += CptAlarme2;
        FausseAlarme3  += CptAlarme3;
        FausseAlarme4  += CptAlarme4;
        CptAlarme1  = 0;
        CptAlarme2  = 0;
        CptAlarme3  = 0;
        CptAlarme4  = 0;
        Serial.print(F("fausse alarmes : ")), Serial.print(FausseAlarme1);
        Serial.print(F(", ")), Serial.print(FausseAlarme2);
        Serial.print(F(", ")), Serial.print(FausseAlarme3);
        Serial.print(F(", ")), Serial.println(FausseAlarme4);
        Acquisition();	//V2-14 on lance Sirene et SMS directement sans attendre prochaine boucle
      }
    }
    else {
      CptAlarme1 = 0; // si batterie coupée efface alarme
      CptAlarme2 = 0; // si batterie coupée efface alarme
      CptAlarme3 = 0; // si batterie coupée efface alarme
      CptAlarme4 = 0; // si batterie coupée efface alarme
    }
  }
  digitalWrite(led, HIGH);	// voyant activité
  Alarm.delay(20);
  digitalWrite(led, LOW);
}
//---------------------------------------------------------------------------
void Acquisition() {
  #ifdef Analyse_boucle
  Serial.print("temps boucle :"),Serial.println(tbouclemoyen);
  cptboucle = 0;
  tbouclemoyen =0;
  #endif
  // ************************ boucle acquisition  ***************************
  //	boucle acquisition
  static int cptRegStatusFault = 0;
  Serial.print(F("Connexion reseau:")),Serial.println(modem.isNetworkConnected());
  Serial.print(F("Reg status      :")),Serial.println(modem.getRegistrationStatus());
  Serial.print(F("Connexion GPRS  :")),Serial.println(modem.isGprsConnected());
  // Patch Blocage modem
  if(modem.getRegistrationStatus() != 1 && modem.getRegistrationStatus() != 5){
    if(cptRegStatusFault ++ > config.cptAla){
      cptRegStatusFault = 0;
      NbrResetModem +=1;
      Serial.println(F("Reset modem suite Reg status fault"));
      modem.send_AT(F("+CRESET"));
      delay(10000);
    }
  }

  Serial.print(F("speed=")), Serial.println(speed);
  displayTime(false);
  
  // NE PAS SUPPRIMER CETTE LIGNE
  // VerifSIM();	// V2-14  verif si SIM OK // 4G
  // incomprehensible, si supprimée, MQTT KO

  static byte nalaTension = 0;
  static byte nRetourTension = 0; //V1-16
  TensionBatterie = map(moyenneAnalogique(), 0, 1023, 0, CoeffTension);//3088, X4573=3029, X4545=3128 V2-122
  //Serial.print(F("Tension batterie = ")), Serial.println(TensionBatterie);
  // Regulateur Solaire coupe à 11.2V
  if (TensionBatterie < 1162 ) {// V1-16 V1-12 || TensionBatterie > 1440
    nalaTension ++;
    if (nalaTension == 4) {
      FlagAlarmeTension = true;
      nalaTension = 0;
    }
  }
  else if (TensionBatterie > 1242) {	//V1-16 hysteresis et tempo sur Alarme Batterie
    nRetourTension ++;
    if (nRetourTension == 4) {
      FlagAlarmeTension = false;
      nRetourTension = 0;
    }
  }
  else {
    //FlagAlarmeTension = false;	// V1-16
    if (nalaTension > 0)nalaTension--;			//	efface progressivement le compteur
  }
  static byte nalaPIR1 = 0;
  static byte nalaPIR2 = 0;
  static byte nalaPIR3 = 0;
  static byte nalaPIR4 = 0;
  if (config.Intru) {
    // gestion des capteurs coupé ou en alarme permanente
    // verif sur 3 passages consecutifs
    if (config.PirActif[0] && digitalRead(Ip_PIR1)) {
      nalaPIR1 ++;
      if (nalaPIR1 > 3) {
        CptAlarme1 = 1;
        FausseAlarme1 = 1000;//V2-11
        FlagPIR = true;
        nalaPIR1 = 0;
      }
    }
    else {
      if (nalaPIR1 > 0) nalaPIR1 --;			//	efface progressivement le compteur
    }

    if (config.PirActif[1] && digitalRead(Ip_PIR2)) {
      nalaPIR2 ++;
      if (nalaPIR2 > 3) {
        CptAlarme2 = 1;
        FausseAlarme2 = 1000;//V2-11
        FlagPIR = true;
        nalaPIR2 = 0;
      }
    }
    else {
      if (nalaPIR2 > 0) nalaPIR2 --;			//	efface progressivement le compteur
    }


    if (config.PirActif[2] && digitalRead(Ip_PIR3)) {
      nalaPIR3 ++;
      if (nalaPIR3 > 3) {
        CptAlarme3 = 1;
        FausseAlarme3 = 1000;//V2-11
        FlagPIR = true;
        nalaPIR3 = 0;
      }
    }
    else {
      if (nalaPIR3 > 0) nalaPIR3 --;			//	efface progressivement le compteur
    }

    if (config.PirActif[3] && digitalRead(Ip_PIR4)) {
      nalaPIR4 ++;
      if (nalaPIR4 > 3) {
        CptAlarme4 = 1;
        FausseAlarme4 = 1000;//V2-11
        FlagPIR = true;
        nalaPIR4 = 0;
      }
    }
    else {
      if (nalaPIR4 > 0) nalaPIR4 --;			//	efface progressivement le compteur
    }
    // gestion des capteurs coupé en alarme permanente

    if (TensionBatterie > 1150) {	// V1-12 seulement si Batterie OK
      if (FlagPIR) {
        FlagAlarmeIntrusion = true;	// Si alarme intrusion active et intrusion detectée
        FlagPIR = false;
        ActivationSonnerie();			// activation Sonnerie
        Serial.println(F("Alarme Intrusion"));
      }
      Serial.print(F("Compte tempo = ")), Serial.println(timecompte);
    }
  }
  else {
    FlagPIR = false;	// efface alarme pendant phase de démarrage
    // V1-12 - V1-13
    CptAlarme1 = 0;
    CptAlarme2 = 0;
    CptAlarme3 = 0;
    CptAlarme4 = 0;
    // V1-12
  }

  // verification index new SMS en attente(raté en lecture directe)
  int smsnum = modem.newMessageIndex(0); // verifie index arrivée sms, -1 si pas de sms
  Serial.print(F("Index last SMS = ")), Serial.println (smsnum);

  if (smsnum > 0) {	// index du SMS en attente
    // il faut les traiter
    traite_sms(smsnum);// 51 demande traitement de tous les SMS en attente
  } else if(smsnum == 0){
    traite_sms(smsnum);// demande traitement du SMS en attente
  }
  else if (smsnum < 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
    FlagReset = false;
    softReset();					//	redemarrage Arduino
  }

  Serial.print(F("freeRAM=")), Serial.println(freeRam());
  // V1-12 - V1-13
  if(config.Intru){
    Serial.print(F("CptTempo = ")), Serial.print(timecompte);
    Serial.print(F(", CptAlarme M1 = ")), Serial.print(CptAlarme1);
    Serial.print(F(" M2 = ")), Serial.print(CptAlarme2);
    Serial.print(F(" R1 = ")), Serial.print(CptAlarme3);
    Serial.print(F(" R2 = ")), Serial.println(CptAlarme4);
  }
  // V1-12
  if (!config.Intru && config.tracker && lancement) {
    // mqttConnect(); // Call the loop to maintain connection to the server. SUPPRIMé Redondant
    gereCadence();
    static byte nalaGprs = 0;
    static byte nalaGps  = 0;
    static byte nalaMQTT = 0;
    if (AlarmeGprs) {
      if (nalaGprs ++ > config.cptAla) {
        FlagAlarmeGprs = true;
        nalaGprs = 0;
      }
    } else {
      if (nalaGprs > 0) {
        nalaGprs --;
      } else {
        FlagAlarmeGprs = false;
      }
    }
    if (AlarmeGps) {
      if (nalaGps ++ > config.cptAla) {
        FlagAlarmeGps = true;
        nalaGps = 0;
      }
    } else {
      if (nalaGps > 0) {
        nalaGps --;
      } else {
        FlagAlarmeGps = false;
      }
    }
    if (AlarmeMQTT) {
      if (nalaMQTT ++ > config.cptAla) {
        FlagAlarmeMQTT = true;
        nalaMQTT = 0;
      }
      Serial.print(F("AlarmeMQTT: ")),Serial.println(nalaMQTT);
    } else {
      // if (nalaMQTT > 0) {
      //   nalaMQTT --;
      // } else {
        FlagAlarmeMQTT = false;
        FlagAlarmeGprs = false;
        nalaMQTT = 0;
      // }
    }
  }

  envoie_alarme();

  // *********************fin boucle acquisition  ***************************
}
//---------------------------------------------------------------------------
void traite_sms(int slot) {	// traitement du SMS par slot
  /* il y a 50 slots dispo
  	si slot=51, demande de balayer tous les slots pour verification
  	si slot=99, demande depuis liaison serie en test, traiter sans envoyer de sms	*/
  Sms smsstruct;
  Serial.print(F("slot: ")); Serial.println(slot);
  // uint16_t smslen;
  // String smsstruct.message;
  // char smsstruct.sendernumber[13];  //we'll store the SMS sender number in here
  // char nameIDbuffer[15];  	//nom expediteur SMS si existe dans Phone Book
  // char datesmsbuffer[21];   //dateheure du sms
  String nomAppelant;  	//nom expediteur SMS si existe dans Phone Book
  byte i;
  byte j;
  bool sms = true;
  static int tensionmemo = 0;	//	memorisation tension batterie lors de la calibration V2-122
  if (slot == 99) sms = false;
  if (slot == 51) { // demande de traitement des SMS en attente
    i = 0; // slot commence à 0 SIM7600
    j = 10;//50
  }
  else {
    i = slot;
    j = slot;
  }
  for (byte k = i; k <= j; k++) {
    slot = k;
    // /* Retrieve SMS sender address/phone number. */
    if (sms) {
      if (!modem.readSMS(&smsstruct,slot)){
        Serial.print(F("Didn't find SMS message in slot!"));
        Serial.println(slot);
        continue;	//	Next k
      }
      if(! Cherche_N_PB(smsstruct.sendernumber)){
        Serial.println(F("Appelant inconnu"));
      } else {
        Serial.print(F("Num :")), Serial.print(Phone.number);
        Serial.print(F(", Nom :")), Serial.println(Phone.text);
      }
      nomAppelant = Phone.text;
      Serial.print(F("Nom appelant:")), Serial.println(nomAppelant);
      
    } else {smsstruct.message = String(message);}
      if(Phone.number.length() < 8){ // numero service free
        for (byte Index = 1; Index < 10; Index++) { // Balayage des Num Tel dans Phone Book
          Phone = {"",""};
          if(modem.readPhonebookEntry(&Phone, Index)){
            if(Phone.number.length() > 0){
              if (config.Pos_Pn_PB[Index] == 1) { // Num dans liste restreinte            
                message = smsstruct.message;
                sendSMSReply(Phone.number , true);
                if (modem.deleteSmsMessage(slot,0)) {
                  Serial.print(F("message supprime, slot=")), Serial.println(slot);
                } else {
                  Serial.print(F("Impossible de supprimer slot=")), Serial.println(slot);
                  modem.deleteSmsMessage(0,4); // efface tous les sms
                }
                return; // sortir de la procedure traite_sms
              }
            } else {Index = 10;}
          }
        }
      }
    
    Serial.print(F("texte du SMS :")), Serial.println(smsstruct.message);
    // for (byte i = 0; i < smsstruct.message.length(); i++) {
      // if ((int)smsstruct.message[i] < 0 || (int)smsstruct.message[i] > 127) { // caracteres accentués interdit
        // goto sortir;
      // }
    // }
    /* Suppression du SMS */
    if (sms) {
      if (modem.deleteSmsMessage(slot,0)) {
        Serial.print(F("message supprime, slot=")), Serial.println(slot);
      } else {
        Serial.print(F("Impossible de supprimer slot=")), Serial.println(slot);
        modem.deleteSmsMessage(0,4); // efface tous les sms
      }
    }
    if ((sms && nomAppelant.length() > 0) || !sms) { // nom appelant existant
      //Envoyer une réponse
      //Serial.println(F("Envoie reponse..."));
      messageId();
      if (!(smsstruct.message.indexOf(F("TEL")) == 0 || smsstruct.message.indexOf(F("tel")) == 0 || smsstruct.message.indexOf(F("Tel")) == 0
            || smsstruct.message.indexOf(F("GPRSDATA")) > -1 || smsstruct.message.indexOf(F("MQTTDATA")) > -1
            || smsstruct.message.indexOf(F("MQTTSERVEUR")) > -1)) {
        smsstruct.message.toUpperCase();		// passe tout en Maj sauf si "TEL"
        smsstruct.message.replace(" ", "");	// supp tous les espaces
      }
      else {}
      // Serial.print(F("texte du SMS txt =")), Serial.println(smsstruct.message);
      if (smsstruct.message.indexOf(F("??")) == 0) {	//	Aide "??"
        message += F("Liste des commandes :");
        message += fl ;
        message += F("Etat      : ETAT/ST");
        message += fl;
        message += F("Etat SYS  : SYS");
        message += fl;
        message += F("Reset Sys : RST");
        message += fl;
        message += F("Alarme Intrusion ON/OFF");
        message += fl;
        message += F("A/D 4500");
        // sendSMSReply(smsstruct.sendernumber, sms);	// SMS n°1
        message += fl;
        // message  = Id;
        message += F("Param Sonnerie: SONN");
        message += fl;
        message += F("SONN=xx:yy:zz (s)");
        message += fl;
        message += F("Alarme Silencieuse ON/OFF");
        message += fl;
        message += F("SilenceON/OFF");
        message += fl;
        message += F("Message Vie");				//Heure du message signe de vie
        message += fl;
        message += F("Vie = 1-24 (H en s)");	// de 0 à 24H en secondes
        // sendSMSReply(smsstruct.sendernumber, sms);	// SMS n°2
        message += fl;
        // message  = Id;
        message += F("Nouvel Id : Id= (max 10c)");
        message += fl;
        message += F("List Num Tel:LST?");
        message += fl;
        message += F("Nouveau Num Tel: ");
        message += fl;
        message += F("Tel=+33612345678,") ;
        message += fl;
        message += F("Nom(max 14c)");
        sendSMSReply(smsstruct.sendernumber, sms);	// SMS n°3
      }
      else if (smsstruct.message.indexOf(F("SPEED=")) >= 0) { // debug speed for localisation
        speed = smsstruct.message.substring(6).toFloat();
        Serial.print(F("saisie.speed=")), Serial.println(speed);
        if(speed > 0){
          flagtestspeed = true;
        } else {flagtestspeed = false;}
      }
      else if (    smsstruct.message.indexOf(F("TEL")) == 0
                || smsstruct.message.indexOf(F("Tel")) == 0
                || smsstruct.message.indexOf(F("tel")) == 0) { // entrer nouveau num
        // TEL=+33123456789,Nom   ajoute N° à la suite
        // TELX=+33123456789,Nom  remplace le N° en X
        // TELX=EFFACE            efface le N° en X

        String numero;
        String nom;
        byte index = 0;
        bool FlagOK = true;
        byte j = 0;
        bool add = true; // ajouter/modification numero = true, suppression = false
        // String sendAT	= F("AT+CPBW=");	// ecriture dans le phone book
        if (smsstruct.message.indexOf(char(61)) == 4) { // TELn= reserver correction/suppression
          int i = smsstruct.message.substring(3).toInt();// recupere n° de ligne
          i = i / 1; // important sinon i ne prend pas sa valeur dans les comparaison?
          //Serial.println(i);
          if (i < 1) FlagOK = false;
          // sendAT += i;
          index = i;
          j = 5;
          // on efface la ligne sauf la 1 pour toujours garder au moins un numéro
          if ( (i != 1) && ( smsstruct.message.indexOf(F("efface")) == 5
                          || smsstruct.message.indexOf(F("EFFACE")) == 5 )){
            add = false;
            goto fin_tel;
          }
        }
        else if (smsstruct.message.indexOf(char(61)) == 3) { // TEL= nouveau numero
          j = 4;
        }
        else {
          FlagOK = false;
        }
        if (smsstruct.message.indexOf("+") == j) {			// debut du num tel +
          if (smsstruct.message.indexOf(",") == j + 12) {	// verif si longuer ok
            numero = smsstruct.message.substring(j, j + 12);
            nom    = smsstruct.message.substring(j + 13, j + 27);	// pas de verif si long<>0?
            // sendAT += F(",\"");
            // sendAT += numero;
            // sendAT += F("\",145,\"");
            // sendAT += nom;
            // sendAT += F("\"");
          }
          else {
            FlagOK = false;
          }
        }
        else {
          FlagOK = false;
        }
fin_tel:
        if (!FlagOK) { // erreur de format
          //Serial.println(F("false"));
          messageId();
          message += F("Commande non reconnue ?");// non reconnu
          sendSMSReply(smsstruct.sendernumber, sms);						// SMS non reconnu
        }
        else {          
          messageId();
          if(add){
            // ajouter N° à index
            message += F("Nouveau Num Tel: ");
            Serial.println(numero);
            Serial.println(nom);
            Serial.println(index);
            if(modem.addPhonebookEntry(numero,nom,index)){
              message += F("OK");
            } else { message += F("KO");}
            message += fl;
            message += index;
            message += ":";
            message += numero;
            message += ":";
            message += nom;
            message += fl;
          } else {
            // suppression N° à Index
            message += F("Suppression Num index: ");
            message += String(index);
            if(modem.deletePhonebookEntry(index)){
              message += F(" OK");
            } else { message += F(" KO");}
          }
          sendSMSReply(smsstruct.sendernumber, sms);
        }
      }
      else if (smsstruct.message == F("LST?") || smsstruct.message == F("LST") || smsstruct.message == F("LST1")) {	//	Liste des Num Tel
        messageId();
        for (byte i = 1; i < 10; i++) {
          Phone = {"",""};
          // Serial.print("lecture PB:"),Serial.println(i);
          if(modem.readPhonebookEntry(&Phone, i)){
            message += String(i) + ":";
            message += Phone.number;
            message += ",";
            message += Phone.text;
            message += "\n";
          } else {
            i = 10;
          }
        }
        sendSMSReply(smsstruct.sendernumber, sms);// envoi sur plusieurs SMS
      }
      else if (smsstruct.message.indexOf(F("ETAT")) == 0 || smsstruct.message.indexOf(F("ST")) == 0) {			// "ETAT? de l'installation"
        generationMessage();
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("SYS")) == 0) {					//	SYS? Etat Systeme
        messageId();
        flushSerial();
        message += modem.getOperator(); // Operateur
        byte n = modem.getRegistrationStatus();        
        if (n == 5) {														// Operateur roaming
          message += F(", rmg, ");								// roaming
        }
        message += " ";
        message += ConnectedNetwork();
        message += fl;

        read_RSSI();														// info RSSI seront ajoutées à message

        if (decodeGPS()) {
          message += F("GPS OK");
          message += fl;
        }
        else {
          message += F("GPS KO");
          message += fl;
        }
        message += F("Param Sonn = ");
        message += config.Dsonn;
        message += ":";
        message += config.DsonnMax;
        message += ":";
        message += config.Dsonnrepos;
        message += "(s)";
        message += fl;
        message += F("freeRAM=");
        message += String(freeRam()) + fl ;
        message += F("Ver:");
        message += ver;
        message += fl;
        message += F("V Batt Sol= ");
        message += String(TensionBatterie / 100) + ",";
        if ((TensionBatterie - (TensionBatterie / 100) * 100) < 10) { //correction bug decimal<10
          message += "0";
        }
        message += TensionBatterie - ((TensionBatterie / 100) * 100);
        message += "V, ";
        //V2-15
        message += String(Battpct(TensionBatterie));
        message += " %";
        //V2-15
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("ID=")) == 0) {			//	Id= nouvel Id
        String temp = smsstruct.message.substring(3);
        if (temp.length() > 0 && temp.length() < 11) {
          Id = "";
          temp.toCharArray(config.Idchar, 11);
          sauvConfig();															// sauvegarde en EEPROM
          Id = String(config.Idchar);
          Id += fl;
        }
        messageId();
        message += F("Nouvel Id");
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("LOG")) == 0) {	// demande log des 5 derniers commandes
        message = "";
        for (int i = 0; i < 5; i++) {
          message += String(record[i].dt) + "," + String(record[i].Act) + "," + String(record[i].Name) + fl;
        }
        //Serial.println( message);
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("INTRUON")) == 0 || smsstruct.message.indexOf(("A" + Id.substring(6, 10))) == 0) {	//	Armement Alarme
        // conserver INTRUON en depannage si ID non conforme
        if (config.tracker) {
          senddata();
          Alarm.delay(1000);// laissé le temps envoyer data avant coupure
          ArretLocalisation();
        }
        if (!config.Intru) {
          config.Intru = !config.Intru;
          sauvConfig();												// sauvegarde en EEPROM
          AllumeCapteur();										// allumage des capteurs selon parametres
          // V1-12 on attache les interruptions
          if (config.PirActif[0]) attachInterrupt(digitalPinToInterrupt(Ip_PIR1), IRQ_PIR1, RISING);
          if (config.PirActif[1]) attachInterrupt(digitalPinToInterrupt(Ip_PIR2), IRQ_PIR2, RISING);
          if (config.PirActif[2]) attachInterrupt(digitalPinToInterrupt(Ip_PIR3), IRQ_PIR3, RISING);
          if (config.PirActif[3]) attachInterrupt(digitalPinToInterrupt(Ip_PIR4), IRQ_PIR4, RISING);
          if (!sms) {															//V2-14
            // Sbidon = F("console");
            // Sbidon.toCharArray(nameIDbuffer, 8);	//	si commande locale
            nomAppelant = F("console");
          }
          logRecord(nomAppelant, "A");					// V2-14 renseigne le log
        }
        generationMessage();
        sendSMSReply(smsstruct.sendernumber, sms);
        //if(!sms){//V2-14
        //Sbidon = F("console");
        //Sbidon.toCharArray(nameIDbuffer,8);	//	si commande locale
        //}
        //logRecord(nameIDbuffer,"A");					// V2-14 renseigne le log
      }
      else if (smsstruct.message.indexOf(F("INTRUOFF")) == 0
               || smsstruct.message.indexOf(("D" + Id.substring(6, 10))) == 0) { //	Desarmement
        if (config.Intru) {
          config.Intru = !config.Intru;
          sauvConfig();															// sauvegarde en EEPROM
          /*	Arret Sonnerie au cas ou? sans envoyer SMS */
          digitalWrite(S_Son, LOW);		// Arret Sonnerie
          Alarm.disable(TSonn);				// on arrete la tempo sonnerie
          Alarm.disable(TSonnMax);		// on arrete la tempo sonnerie maxi
          // V1-12 on detache les interruptions
          if (config.PirActif[0]) detachInterrupt(digitalPinToInterrupt(Ip_PIR1));
          if (config.PirActif[1]) detachInterrupt(digitalPinToInterrupt(Ip_PIR2));
          if (config.PirActif[2]) detachInterrupt(digitalPinToInterrupt(Ip_PIR3));
          if (config.PirActif[3]) detachInterrupt(digitalPinToInterrupt(Ip_PIR4));
          digitalWrite(Op_PIR1, LOW); // on etteint les capteurs PIR TX et RX
          digitalWrite(Op_PIR2, LOW);
          digitalWrite(Op_PIR3, LOW);
          digitalWrite(Op_PIR4, LOW);
          FirstSonn = false;
          FlagAlarmeIntrusion = false;
          FlagPIR = false;
          if (!sms) {															//V2-14
            // Sbidon = F("console");
            // Sbidon.toCharArray(nameIDbuffer, 8);	//	si commande locale
            nomAppelant = F("console");
          }
          logRecord(nomAppelant, "D");					// V2-14 renseigne le log
        }
        FlagAlarmeGprs = false;
        FlagAlarmeMQTT = false;
        generationMessage();
        sendSMSReply(smsstruct.sendernumber, sms);

        if (config.tracker){
          senddata(); // active localisation
          Alarm.write(Send, config.trapide);
          Accu = 255;
        }
        //if(!sms){//V2-14
        //Sbidon = F("console");
        //Sbidon.toCharArray(nameIDbuffer,8);	//	si commande locale
        //}
        //logRecord(nameIDbuffer,"D");					// V2-14 renseigne le log
      }
      else if (smsstruct.message.indexOf(F("SILENCE")) == 0 ) {		//	Alarme Silencieuse
        if (smsstruct.message.indexOf(F("ON")) == 7) { //ON
          if (!config.Silence) {
            config.Silence = !config.Silence;
            // V2-17
            /*	Arret Sonnerie au cas ou? sans envoyer SMS */
            digitalWrite(S_Son, LOW);	// Arret Sonnerie
            // ne pas arreter les tempos risque bug blocagE Alarme
            // Alarm.disable(TSonn);			// on arrete la tempo sonnerie
            // Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
            // V2-17
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
        if (smsstruct.message.indexOf(F("OFF")) == 7) {
          if (config.Silence) {
            config.Silence = !config.Silence;
            sauvConfig();															// sauvegarde en EEPROM
            // V2-17
            /*	Arret Sonnerie au cas ou? sans envoyer SMS */
            // digitalWrite(S_Son, LOW);	// Arret Sonnerie
            // Alarm.disable(TSonn);			// on arrete la tempo sonnerie
            // Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
            // V2-17
          }
        }
        generationMessage();
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("CAPTEUR")) == 0) {			// Capteurs actif CAPTEUR=1,0,1,1
        if (smsstruct.message.indexOf(char(61)) == 7) { //char(61) "="	liste capteur actif
          byte Num[4];
          Sbidon = smsstruct.message.substring(8, 15);
          Serial.print(F("Sbidon=")), Serial.print(Sbidon), Serial.println(Sbidon.length());
          if (Sbidon.length() == 7) {
            int j = 0;
            for (int i = 0; i < 7; i += 2) {
              if (Sbidon.substring(i, i + 1) == "0" || Sbidon.substring(i, i + 1) == "1") {
                // Serial.print(",="),Serial.println(Sbidon.substring(i+1,i+2));
                // Serial.print("X="),Serial.println(Sbidon.substring(i,i+1));
                Num[j] = Sbidon.substring(i, i + 1).toInt();
                // Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(Num[j]);
                j++;
              }
              else {
                Serial.println(F("pas bon"));
                goto FinCapteur;	// pas bon on sort
              }
            }
            // copie des num
            for (int i = 0 ; i < 4 ; i++) {
              config.PirActif[i] = Num[i];
              Serial.print(Num[i]);
            }
            Serial.println();
            sauvConfig();
            if(config.Intru){ // V4
              AllumeCapteur();
              if (config.PirActif[0]) { // V2-12
                attachInterrupt(digitalPinToInterrupt(Ip_PIR1), IRQ_PIR1, RISING);
              }
              else {
                detachInterrupt(digitalPinToInterrupt(Ip_PIR1));
              }
              if (config.PirActif[1]) {
                attachInterrupt(digitalPinToInterrupt(Ip_PIR2), IRQ_PIR2, RISING);
              }
              else {
                detachInterrupt(digitalPinToInterrupt(Ip_PIR2));
              }
              if (config.PirActif[2]) {
                attachInterrupt(digitalPinToInterrupt(Ip_PIR3), IRQ_PIR3, RISING);
              }
              else {
                detachInterrupt(digitalPinToInterrupt(Ip_PIR3));
              }
              if (config.PirActif[3]) {
                attachInterrupt(digitalPinToInterrupt(Ip_PIR4), IRQ_PIR4, RISING);
              }
              else {
                detachInterrupt(digitalPinToInterrupt(Ip_PIR4));
              }
            }
          }
        }
FinCapteur:
        message += F("Capteurs actifs");
        message += fl;
        for (int i = 0; i < 4; i++) {
          message += String(config.PirActif[i]);
          if (i < 3) message += ",";
        }
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("FIRST")) == 0 ) {		//	premier message lancement
        if (smsstruct.message.indexOf(F("ON")) == 5) {
          if (!config.Pos_PN) {
            config.Pos_PN = !config.Pos_PN;
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
        if (smsstruct.message.indexOf(F("OFF")) == 5) {
          if (config.Pos_PN) {
            config.Pos_PN = !config.Pos_PN;
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
        if (config.Pos_PN) {
          message += F("First message ON");						//Premier message ON
        }
        else {
          message += F("First message OFF"); 					//Premier message OFF
        }
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("LST2")) == 0 ) {		//	Liste restreinte Info PN Fermé	//  =LST2=0,1,0,0,0,0,0,0,0,
        if (smsstruct.message.indexOf(char(61)) == 4) { //char(61) "="
          byte Num[10];
          Sbidon = smsstruct.message.substring(5, 23);
          //Serial.println("Sbidon="),Serial.print(Sbidon),Serial.println(Sbidon.length());
          if (Sbidon.length() == 18) { //18
            int j = 1;
            for (int i = 0; i < 18; i += 2) {		//18
              if ((Sbidon.substring(i + 1, i + 2) == ",") && (Sbidon.substring(i, i + 1) == "0"	|| Sbidon.substring(i, i + 1) == "1")) {
                //Serial.print(",="),Serial.println(Sbidon.substring(i+1,i+2));
                //Serial.print("X="),Serial.println(Sbidon.substring(i,i+1));
                Num[j] = Sbidon.substring(i, i + 1).toInt();
                //Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(Num[j]);
                j++;
              }
              else {
                Serial.println(F("pas bon"));
                goto FinLSTPOSPN;	// pas bon on sort
              }
            }
            //Serial.println("copie des num");
            for (int i = 1; i < 10; i++) {
              config.Pos_Pn_PB[i] = Num[i];
            }
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
FinLSTPOSPN:
        message += F("Liste restreinte");
        message += fl;
        for (int i = 1; i < 10; i++) {
          message += config.Pos_Pn_PB[i];
          if ( i < 9) message += char(44);						// ,
        }
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("VIE")) == 0) {			//	Heure Message Vie
        //if ((smsstruct.message.indexOf("?") == 3) || (smsstruct.message.indexOf(char(61))) == 3) { //char(61) "=" V2-14
        if ((smsstruct.message.indexOf(char(61))) == 3) {
          long i = atol(smsstruct.message.substring(4).c_str()); //	Heure message Vie
          if (i > 0 && i <= 86340) {										//	ok si entre 0 et 86340(23h59)
            config.Ala_Vie = i;
            sauvConfig();																// sauvegarde en EEPROM
            Svie = Alarm.alarmRepeat(config.Ala_Vie, SignalVie);	// init tempo
          }
        }
        message += F("Heure Vie = ");
        message += int(config.Ala_Vie / 3600);
        message += ":";
        message += int((config.Ala_Vie % 3600) / 60);
        message += F("(hh:mm)");
        //message += fl;
        sendSMSReply(smsstruct.sendernumber, sms);
        //}
      }
      else if (smsstruct.message.indexOf(F("PARAM")) == 0) {				//	Parametres fausses alarmes
        // V2-122
        if (smsstruct.message.indexOf(char(61)) == 5) { //"="
          int x = smsstruct.message.indexOf(":");
          int y = smsstruct.message.indexOf(":", x + 1);
          int z = smsstruct.message.indexOf(":", y + 1);
          int i = atoi(smsstruct.message.substring(6, x).c_str());		// Jour nombre de fausses Alarmes
          int j = atoi(smsstruct.message.substring(x + 1, y).c_str());	// duree analyse
          int k = atoi(smsstruct.message.substring(y + 1, z).c_str());	// Nuit nombre de fausses Alarmes
          int l = atoi(smsstruct.message.substring(z + 1).c_str());	// duree analyse
          if (i > 0 && i < 101 && j > 9 && j < 601 &&
              k > 0 && k < 101 && l > 9 && l < 601) {
            // nombre entre 1 et 100, durée entre 10 et 600
            config.Jour_Nmax 		 = i;
            config.Jour_TmCptMax = j; //passage en s
            config.Nuit_Nmax 		 = k;
            config.Nuit_TmCptMax = l; //passage en s
            sauvConfig();													// sauvegarde en EEPROM V2-12
          }
        }
        message += F("Parametres Fausses Alarmes");
        message += fl;
        message += F("Jour n = ");
        message += config.Jour_Nmax;
        message += F(", t = ");
        message += config.Jour_TmCptMax;
        message += F("(s)");
        message += fl;
        message += F("Nuit n = ");
        message += config.Nuit_Nmax;
        message += F(", t = ");
        message += config.Nuit_TmCptMax;
        message += F("(s)");
        message += fl;
        message += F("Actuel n = ");
        message += Nmax;
        message += F(", t = ");
        message += TmCptMax;
        message += F("(s)");
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("FALARME")) == 0) { // fausses alarmes V2-21
        MessageFaussesAlarmes(0);
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("RSTFALARME")) == 0) { // Reset compteurs fausses alarmes V4
        MessageFaussesAlarmes(2);
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("HINTRU")) == 0 || smsstruct.message.indexOf(F("HPARAM")) == 0 ) {	//V2-122	Heures chagement parametre
        if (smsstruct.message.indexOf(char(61)) == 6) {	//	"=" changement heure Intru Auto
          // Hintru=Hsoir,Hmatin; Hintru=75600,21600
          int  x = smsstruct.message.indexOf(",");
          long i = atol(smsstruct.message.substring(7, x).c_str());	// valeur Soir
          long j = atol(smsstruct.message.substring(x + 1).c_str());		// valeur Matin
          if (i > 0 && i <= 86340 &&
              j > 0 && j <= 86340); { //	ok si i entre 0 et 86340(23h59) et > Heure matin 	ok si j entre 0 et 86340(23h59) et < Heure soir
            if (config.IntruDebut != i || config.IntruFin != j) { // si changement
              config.IntruDebut 	= i;
              config.IntruFin 		= j;

              sauvConfig();							// sauvegarde en EEPROM
              Alarm.disable(HIntruD);		// on arrete les alarmes
              Alarm.disable(HIntruF);
              Alarm.write(HIntruF, config.IntruFin);
              Alarm.write(HIntruD, config.IntruDebut);
              // HIntruF = Alarm.alarmRepeat(config.IntruFin, IntruF);// on parametre
              // HIntruD = Alarm.alarmRepeat(config.IntruDebut , IntruD);
              // Alarm.enable(HIntruD);		// on redemarre les alarmes
              // Alarm.enable(HIntruF);
              AIntru_HeureActuelle();
            }
          }
        }
        message += F("Parametre Auto");
        message += fl;
        message += F("debut nuit : ");
        message += int(config.IntruDebut / 3600);
        message += ":";
        message += int((config.IntruDebut % 3600) / 60);
        message += F("(hh:mm)");
        message += fl;
        message += F("fin nuit     : ");
        message += int(config.IntruFin / 3600);
        message += ":";
        message += int((config.IntruFin % 3600) / 60);
        message += F("(hh:mm)");

        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("CALIBRATION=")) == 0) {
        /* 	Mode calibration mesure tension V2-122
        		Seulement en mode serie local
        		recoit message "CALIBRATION=0000"
        		entrer mode calibration
        		effectue mesure tension avec CoeffTensionDefaut retourne et stock resultat
        		recoit message "CALIBRATION=1250" mesure réelle en V*100
        		calcul nouveau coeff = mesure reelle/resultat stocké * CoeffTensionDefaut
        		applique nouveau coeff
        		stock en EEPROM
        		sort du mode calibration

        		variables
        		FlagCalibration true cal en cours, false par defaut
        		static int tensionmemo memorisation de la premiere tension mesurée en calibration
        		int CoeffTension = CoeffTensionDefaut 3100 par défaut
        */
        Sbidon = smsstruct.message.substring(12, 16);
        //Serial.print(F("Sbidon=")),Serial.print(Sbidon),Serial.print(","),Serial.println(Sbidon.length());
        if (Sbidon.substring(0, 1) == "0" ) { // debut mode cal
          FlagCalibration = true;
          CoeffTension = CoeffTensionDefaut;
          TensionBatterie = map(moyenneAnalogique(), 0, 1023, 0, CoeffTension);
          // Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
          tensionmemo = TensionBatterie;
        }
        else if (FlagCalibration && Sbidon.substring(0, 4).toInt() > 0 && Sbidon.substring(0, 4).toInt() <= 5000) {
          // si Calibration en cours et valeur entre 0 et 5000

          /* calcul nouveau coeff */
          CoeffTension = Sbidon.substring(0, 4).toFloat() / float(tensionmemo) * CoeffTensionDefaut;
          // Serial.print("Coeff Tension = "),Serial.println(CoeffTension);
          TensionBatterie = map(moyenneAnalogique(), 0, 1023, 0, CoeffTension);
          // Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
          FlagCalibration = false;
          Serial.print(F("long EEPROM:"));
          Serial.println(EEPROM_writeAnything(EEPROM_adresse[0], CoeffTension));	// sauvegarde en EEPROM
        }
        message += F("Mode Calib Tension");
        message += fl;
        message += F("TensionBatterie = ");
        message += TensionBatterie;
        message += fl;
        message += F("Coeff Tension = ");
        message += CoeffTension;
        //V2-15
        message += fl;
        message += F("Batterie = ");
        message += String(Battpct(TensionBatterie));
        message += "%";
        //V2-15
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("SONN")) == 0) { //	Durée Sonnerie
        if (smsstruct.message.indexOf(char(61)) == 4) {
          int x = smsstruct.message.indexOf(":");
          int y = smsstruct.message.indexOf(":", x + 1);

          int i = atoi(smsstruct.message.substring(5, x).c_str());			//	Dsonn Sonnerie
          int j = atoi(smsstruct.message.substring(x + 1, y).c_str());	//	DsonnMax Sonnerie
          int k = atoi(smsstruct.message.substring(y + 1).c_str()); 		//	Dsonnrepos Sonnerie
          //Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(k);
          if (i > 5  && i <= 300 &&
              j > i  && j <= 600 &&
              k > 10 && k <= 300) {			//	ok si entre 10 et 300
            config.Dsonn 			= i;
            config.DsonnMax 	= j;
            config.Dsonnrepos = k;
            sauvConfig();																// sauvegarde en EEPROM
          }
        }
        message += F("Param Sonnerie = ");
        message += config.Dsonn;
        message += ":";
        message += config.DsonnMax;
        message += ":";
        message += config.Dsonnrepos;
        message += "(s)";
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("SIRENE")) == 0) { // Lancement SIRENE V2-12
        digitalWrite(S_Son, HIGH);		// Marche Sonnerie
        Alarm.enable(TSonn);					// lancement tempo
        message += F("Lancement Sirene");
        message += fl;
        message += config.Dsonn;
        message += F("(s)");
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("TIME")) == 0 && smsstruct.message.length() == 4) { //	Heure Systeme
        message += F("Heure Sys = ");
        displayTime(true);
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("MAJHEURE")) == 0) {	//	forcer mise a l'heure V2-19
        if (sms) {
          String mytime = smsstruct.timestamp.substring(0, 20);
          // Serial.print(F("heure du sms:")),Serial.println(mytime);
          // String _temp = F("AT+CCLK=\"");
          String _temp = F("+CCLK=\"");
          _temp += mytime + "\"\r\n";
          // Serial.print(_temp);
          modem.send_AT(_temp);
          Alarm.delay(100);
          MajHeure(true);			// mise a l'heure forcée
        }
        else {
          message += F("pas de mise à l'heure en local");
        }
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("POSITION")) == 0) {	// demande position
        // on lance la demande au GPS
        if (decodeGPS()) {
          //http://maps.google.fr/maps?f=q&hl=fr&q=42.8089900,2.2614000
          message += F("http://maps.google.fr/maps?f=q&hl=fr&q=");
          message += String(lat,6);
          message += F(",");
          message += String(lon,6);
          message += fl;
          message += F("Vitesse = ");
          message += String(speed, 1);
          message += F("km/h");
          message += fl;
          message += F("Dir = ");
          message += String(heading, 0);
        }
        else { // si FIX GPS KO
          message += F("GPS pas verrouille !");
        }
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message == F("RST")) {							    // demande RESET
        message += F("Le systeme va etre relance");	// apres envoie du SMS!
        FlagReset = true;														// reset prochaine boucle
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("IMEI")) == 0) {		// V2-17 recuperer IMEI
        // char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
        String modemInfo = modem.getModemInfo(); // IMEI: 862195XXXXXX785
        byte pos = modemInfo.indexOf(F("IMEI:"));
        if (pos > 0) {
          // Serial.print(F("Module IMEI: ")), Serial.println(modemInfo.substring(pos+6, pos+6+15));
          message += F("IMEI = ");
          message += modemInfo.substring(pos+6, pos+6+15);
          sendSMSReply(smsstruct.sendernumber, sms);
        }
      }
      else if (smsstruct.message.indexOf("TIMERLENT") == 0) { //	Timer lent
        if ((smsstruct.message.indexOf(char(61))) == 9) {
          int i = smsstruct.message.substring(10).toInt();
          if (i > 9 && i <= 3601) {								//	ok si entre 10 et 3600
            config.tlent = i;
            sauvConfig();													// sauvegarde en EEPROM
            Alarm.disable(Send);
            // Send = Alarm.timerRepeat(config.tlent, senddata); // send data
            Alarm.write(Send, config.tlent);
          }
        }
        message += F("Timer Lent = ");
        message += String(config.tlent);
        message += fl;
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("TIMERRAPIDE")) == 0) { //	Timer rapide
        if ((smsstruct.message.indexOf(char(61))) == 11) {
          int i = smsstruct.message.substring(12).toInt();
          if (i > 4 && i <= 3601) {								//	ok si entre 5 et 3600
            config.trapide = i;
            sauvConfig();													// sauvegarde en EEPROM
            Alarm.disable(Send);
            // Send = Alarm.timerRepeat(config.trapide, senddata); // Send data
            Alarm.write(Send, config.trapide);
          }
        }
        message += F("Timer Rapide = ");
        message += String(config.trapide);
        message += fl;
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("VITESSEMINI")) == 0) { //	Vitesse mini
        if ((smsstruct.message.indexOf(char(61))) == 11) {
          int i = smsstruct.message.substring(12).toInt();
          if (i > 0 && i <= 21) {								//	ok si entre 1 et 20
            config.vtransition = i;
            sauvConfig();													// sauvegarde en EEPROM
          }
        }
        message += F("Vitesse mini = ");
        message += String(config.vtransition);
        message += fl;
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("TRACKER")) == 0) { // Tracker actif
        if (((smsstruct.message.indexOf(char(61))) == 7) || ((smsstruct.message.indexOf("O")) == 7)) { // = ou O
          int i = 0;
          if (smsstruct.message.substring(7) == F("ON")) {
            i = 1;
          } else if (smsstruct.message.substring(7) == F("OFF")) {
            i = 0;
          } else {
            i = smsstruct.message.substring(8).toInt();
          }
          if (i == 1) {
            config.tracker = true;
            Accu = 255;
            sauvConfig();																// sauvegarde en EEPROM
            if (!config.Intru) {
              Alarm.disable(Send);
              Alarm.write(Send, config.trapide);
              senddata(); // active la localisation
            }
          }
          else if (i == 0) {
            config.tracker = false;
            sauvConfig();																// sauvegarde en EEPROM
            ArretLocalisation();
          }
        }
        message += F("Tracker = ");
        if (config.tracker) {
          message += "1";
        }
        else {
          message += "0";
        }
        message += fl;
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("TRCKPARAM")) > -1) { // Parametre Tracker, Timer lent/rapide, vitesse mini
        /* parametres Tracker en sms:
          TRCKPARAM=timerlent:timerrapide:vitessemini
          {"TRCKPARAM":{"timerlent":600,"timerrapide":15,"vitessemini":2}}
        */
        bool erreur = false;
        bool formatsms = false;
        // Serial.print("position X:"),Serial.println(smsstruct.message.substring(7, 8));
        // Serial.print("position ::"),Serial.println(smsstruct.message.substring(8, 9));
        if (smsstruct.message.substring(12, 13) == ":") {
          DynamicJsonDocument doc(108);
          DeserializationError err = deserializeJson(doc, smsstruct.message);
          if (err) {
            erreur = true;
          }
          else {
            // Serial.print(F("Deserialization succeeded"));
            JsonObject param = doc["TRCKPARAM"];
            config.tlent = param["TIMERLENT"];
            config.trapide = param["TIMERRAPIDE"];
            config.vtransition = param["VITESSEMINI"];
            sauvConfig();
          }
        }
        else if ((smsstruct.message.indexOf(char(61))) == 9) { // format sms
          formatsms = true;
          byte cpt = 0;
          byte i = 10;
          do { // compte nombre de : doit etre =2
            i = smsstruct.message.indexOf(':', i + 1);
            cpt ++;
          } while (i <= smsstruct.message.length());
          if (cpt - 1 == 2) {
            byte x = smsstruct.message.indexOf(':');
            byte y = smsstruct.message.indexOf(':', x + 1);
            int t1 = smsstruct.message.substring(10, x).toInt();
            int t2 = smsstruct.message.substring(x + 1, y).toInt();
            int t3 = smsstruct.message.substring(y + 1).toInt();
            if (t1 > 9 && t1 < 3601 && t2 > 0 && t2 < 1801 && t3 > 0 && t3 < 21) {
              config.tlent = t1;
              config.trapide = t2;
              config.vtransition = t3;
              sauvConfig();
            }
            else {
              erreur = true;
            }
          }
          else {
            erreur = true;
          }
        }
        if (!erreur) {
          if (formatsms) {
            message += F("Tracker parametres\n");
            message += F("timerlent:timerrapide:vitessemini\n");
            message += String(config.tlent) + ":" + String(config.trapide) + ":" + String(config.vtransition);
          }
          else {
            // calculer taille https://arduinojson.org/v6/assistant/
            DynamicJsonDocument doc(108);
            JsonObject param = doc.createNestedObject("param");
            param["timerlent"] = config.tlent;
            param["timerrapide"] = config.trapide;
            param["vitessemini"] = config.vtransition;
            String jsonbidon;
            serializeJson(doc, jsonbidon);
            // serializeJson(doc, Serial);
            message += jsonbidon;
          }
        }
        else {
          message += "erreur format";
        }
        message += fl;
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf("MQTTDATA") > -1) { // Parametres MQTT seul serveur peut etre changé
        // Parametres MQTTDATA=Serveur:User:Pass:Topic:port
        bool erreur = false;
        bool formatsms = false;
        if (smsstruct.message.indexOf(":") == 11) { // format json
          DynamicJsonDocument doc(210); //https://arduinojson.org/v6/assistant/
          DeserializationError err = deserializeJson(doc, smsstruct.message);
          if (err) {
            erreur = true;
          }
          else {
            JsonObject mqttdata = doc["MQTTDATA"];
            strncpy(config.mqttServer,   mqttdata["serveur"], 26);
            strncpy(config.mqttUserName, mqttdata["user"],    11);
            strncpy(config.mqttPass,     mqttdata["pass"],    16);
            strncpy(config.writeTopic,   mqttdata["topic"],   16);
            config.mqttPort = int(mqttdata["port"]);
            sauvConfig();													// sauvegarde en EEPROM
          }
        }
        else if ((smsstruct.message.indexOf(char(61))) == 8) { // format sms
          formatsms = true;
          byte w = smsstruct.message.indexOf(":");
          byte x = smsstruct.message.indexOf(":", w + 1);
          byte y = smsstruct.message.indexOf(":", x + 1);
          byte z = smsstruct.message.indexOf(":", y + 1);
          byte zz = smsstruct.message.length();
          if (smsstruct.message.substring(z + 1, zz).toInt() > 0) { // Port > 0
            if ((w - 9) < 25 && (x - w - 1) < 11 && (y - x - 1) < 16 && (z - y - 1) < 16) {
              Sbidon = smsstruct.message.substring(9, w);
              Sbidon.toCharArray(config.mqttServer, (Sbidon.length() + 1));
              Sbidon = smsstruct.message.substring(w + 1, x);
              Sbidon.toCharArray(config.mqttUserName, (Sbidon.length() + 1));
              Sbidon = smsstruct.message.substring(x + 1, y);
              Sbidon.toCharArray(config.mqttPass, (Sbidon.length() + 1));
              Sbidon = smsstruct.message.substring(y + 1, z);
              Sbidon.toCharArray(config.writeTopic, (Sbidon.length() + 1));
              Sbidon = smsstruct.message.substring(z + 1, zz);
              config.mqttPort = Sbidon.toInt();
              sauvConfig();													// sauvegarde en EEPROM
            }
            else {
              erreur = true;
            }
          } else {
            erreur = true;
          }
        }
        if (!erreur) {
          if (formatsms) {
            message += F("Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant");
            message += fl;
            message += F("Parametres MQTT :");
            message += fl;
            message += F("Serveur:");
            message += String(config.mqttServer) + fl;
            message += F("User:");
            message += String(config.mqttUserName) + fl;
            message += F("Pass:");
            message += String(config.mqttPass) + fl;
            message += F("Topic:");
            message += String(config.writeTopic) + fl;
            message += F("Port:");
            message += String(config.mqttPort) + fl;
          }
          else {
            DynamicJsonDocument doc(210);
            JsonObject MQTTDATA = doc.createNestedObject("MQTTDATA");
            MQTTDATA["serveur"] = config.mqttServer;
            MQTTDATA["user"]    = config.mqttUserName;
            MQTTDATA["pass"]    = config.mqttPass;
            MQTTDATA["topic"]   = config.writeTopic;
            MQTTDATA["port"]    = config.mqttPort;
            Sbidon = "";
            serializeJson(doc, Sbidon);
            message += Sbidon;
            message += fl;
          }
        }
        else {
          message += F("Erreur format");
          message += fl;
        }
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf("MQTTSERVEUR") == 0) { // Serveur MQTT
        // case sensitive
        // MQTTSERVEUR=abcd.org
        if (smsstruct.message.indexOf(char(61)) == 11) {
          Sbidon = smsstruct.message.substring(12);
          Sbidon.toCharArray(config.mqttServer, (Sbidon.length() + 1));
          sauvConfig();
        }
        message += F("MQTTserveur =");
        message += String(config.mqttServer);
        message += F("\n au prochain demarrage");
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("GPRSDATA")) > -1) { // GPRS Apn,User,Pass
        // Parametres GPRSDATA = "APN":"user":"pass"
        // GPRSDATA="sl2sfr":"":""
        // {"GPRSDATA":{"apn":"sl2sfr","user":"","pass":""}}
        bool erreur = false;
        bool formatsms = false;
        if (smsstruct.message.indexOf(":") == 11) { // format json
          DynamicJsonDocument doc(120);
          DeserializationError err = deserializeJson(doc, smsstruct.message);
          if (err) {
            erreur = true;
          }
          else {
            JsonObject gprsdata = doc["GPRSDATA"];
            strncpy(config.apn, gprsdata["apn"], 11);
            strncpy(config.gprsUser, gprsdata["user"], 11);
            strncpy(config.gprsPass, gprsdata["pass"], 11);
            // Serial.print("apn length:"),Serial.println(strlen(gprsdata["apn"]));
            // Serial.print("apn:"),Serial.println(config.apn);
            // Serial.print("user:"),Serial.println(config.gprsUser);
            // Serial.print("pass:"),Serial.println(config.gprsPass);
            sauvConfig();													// sauvegarde en EEPROM
          }
        }
        else if ((smsstruct.message.indexOf(char(61))) == 8) { // format sms
          formatsms = true;
          byte cpt = 0;
          byte i = 9;
          do { // compte nombre de " doit etre =6
            i = smsstruct.message.indexOf('"', i + 1);
            cpt ++;
          } while (i <= smsstruct.message.length());
          // Serial.print("nombre de \" :"), Serial.println(cpt);
          if (cpt == 6) {
            byte x = smsstruct.message.indexOf(':');
            byte y = smsstruct.message.indexOf(':', x + 1);
            byte z = smsstruct.message.lastIndexOf('"');
            // Serial.printf("%d:%d:%d\n",x,y,z);
            // Serial.printf("%d:%d:%d\n", x -1 - 10, y-1 - x-1-1, z - y-1-1);
            if ((x - 11) < 11 && (y - x - 3) < 11 && (z - y - 2) < 11) { // verification longueur des variables
              Sbidon = smsstruct.message.substring(10, x - 1);
              Sbidon.toCharArray(config.apn, (Sbidon.length() + 1));
              Sbidon = smsstruct.message.substring(x + 1 + 1 , y - 1);
              Sbidon.toCharArray(config.gprsUser, (Sbidon.length() + 1));
              Sbidon = smsstruct.message.substring(y + 1 + 1, z);
              Sbidon.toCharArray(config.gprsPass, (Sbidon.length() + 1));

              // Serial.print("apn:"),Serial.println(config.apn);
              // Serial.print("user:"),Serial.println(config.gprsUser);
              // Serial.print("pass:"),Serial.println(config.gprsPass);

              sauvConfig();													// sauvegarde en EEPROM
            }
            else {
              erreur = true;
            }
          }
          else {
            erreur = true;
          }
        }
        if (!erreur) {
          if (formatsms) {
            message += F("Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant");
            message += fl;
            message += F("Parametres GPRS \"apn\":\"user\":\"pass\"");
            message += fl + "\"";
            message += String(config.apn);
            message += "\":\"";
            message += String(config.gprsUser);
            message += "\":\"";
            message += String(config.gprsPass);
            message += "\"" + fl;
          }
          else {
            DynamicJsonDocument doc(120);
            JsonObject gprsdata = doc.createNestedObject("GPRSDATA");
            gprsdata["apn"]  = config.apn;
            gprsdata["user"] = config.gprsUser;
            gprsdata["pass"] = config.gprsPass;
            Sbidon = "";
            serializeJson(doc, Sbidon);
            message += Sbidon;
            message += fl;
          }
        }
        else {
          message += F("Erreur format");
          message += fl;
        }
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("CPTALATRCK")) == 0) { // Compteur Ala avant Flag
        if (smsstruct.message.indexOf(char(61)) == 10) {
          byte c = smsstruct.message.substring(11).toInt();
          if (c > 1 && c < 501) {
            config.cptAla = c;
            sauvConfig();
          }
        }
        message += F("Cpt Ala Tracker (x15s)=");
        message += String(config.cptAla);
        message += fl;
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("SETNETWORKMODE")) >= 0) {// Set Prefered network Mode
        if(smsstruct.message.indexOf(char(61)) == 14){
          int mode = smsstruct.message.substring(15).toInt();
          if(mode == 2 || mode == 13 || mode == 14 || mode == 19 || mode == 38
          || mode == 39 || mode == 51 || mode == 54){
            modem.setNetworkMode(mode);
            delay(1000);
          }
        }
        message += String(modem.send_AT(F("+CNMP?")));
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("SENDAT")) == 0){
        // envoie commande AT au SIM7600
        // ne pas mettre AT au debut, +CPBR=1
        // attention DANGEREUX pas de verification!
        if (smsstruct.message.indexOf(char(61)) == 6) {
          String CdeAT = smsstruct.message.substring(7, smsstruct.message.length());
          message += String(modem.send_AT(CdeAT));
          sendSMSReply(smsstruct.sendernumber, sms);
        }
      }
      else if (smsstruct.message.indexOf(F("MODEMINFO")) == 0){
        // Get Modem Info
        message += modem.getModemInfo();
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else if (smsstruct.message.indexOf(F("CPTRESETMODEM")) == 0){
        // Demande nombre de reset modem
        message += F("Compteur reset Modem : ");
        message += String(NbrResetModem);
        sendSMSReply(smsstruct.sendernumber, sms);
      }else if (smsstruct.message.indexOf(F("NETWORKHISTO")) == 0){
        // Demande Changement etat reseau
        message_Monitoring_Reseau();
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      else {
        message += F("Commande non reconnue ?");		//"Commande non reconnue ?"
        sendSMSReply(smsstruct.sendernumber, sms);
      }
      //Serial.print(message);
    }
    else {
      Serial.print(F("Appelant non reconnu ! ")), Serial.println(smsstruct.sendernumber);
    }
// sortir:
    Serial.println(""); /* indispensable apres xxxx: */
    flushSerial();
  }
}
//---------------------------------------------------------------------------
void envoie_alarme() {
  /* determine si un SMS apparition/disparition Alarme doit etre envoyé */
  bool SendEtat = false;

  if (FlagAlarmeTension != FlagLastAlarmeTension) {
    SendEtat = true;
    FlagLastAlarmeTension = FlagAlarmeTension;
  }
  if (FlagAlarmeIntrusion != FlagLastAlarmeIntrusion) {
    SendEtat = true;
    FlagLastAlarmeIntrusion = FlagAlarmeIntrusion;
  }
  if (FlagAlarmeGprs != FlagLastAlarmeGprs) {
    SendEtat = true;
    FlagLastAlarmeGprs = FlagAlarmeGprs;
  }
  if (FlagAlarmeMQTT != FlagLastAlarmeMQTT) {
    SendEtat = true;
    FlagLastAlarmeMQTT = FlagAlarmeMQTT;
  }
  if (FlagAlarmeGps != FlagLastAlarmeGps) {
    SendEtat = true;
    FlagLastAlarmeGps = FlagAlarmeGps;
  }
  if (SendEtat) { 							// si envoie Etat demandé
    envoieGroupeSMS(0,false);		// envoie groupé
    SendEtat = false;						// efface demande
  }
}
//---------------------------------------------------------------------------
  /* si grp = 0,
    envoie un SMS à tous les numero existant (9 max) du Phone Book
    si grp = 1,
    envoie un SMS à tous les numero existant (9 max) du Phone Book
    de la liste restreinte config.Pos_Pn_PB[x]=1			
    
    vie = true, ajoute au message nombre de reset modem */
void envoieGroupeSMS(byte grp, bool vie) {

  generationMessage();
  if(vie){
    message += F("Reset modem : ");
    message += String(NbrResetModem);
    message += fl;
    message_Monitoring_Reseau();
  }
  for (byte Index = 1; Index < 10; Index++) {		// Balayage des Num Tel Autorisés=1 dans Phone Book
    Phone = {"",""};
    if(modem.readPhonebookEntry(&Phone, Index)){
      Serial.print(Index),Serial.print(","),Serial.println(Phone.number);
      if(Phone.number.length() > 0){
        if (grp == 1){	// grp = 1 message liste restreinte
          if (config.Pos_Pn_PB[Index] == 1) {            
            message += F("lancement");
            sendSMSReply(Phone.number, true);
          }
        } else {	// grp = 0, message à tous
          sendSMSReply(Phone.number , true);
        }        
      } else {
        Index = 10;
      }
    }
  }
}
//---------------------------------------------------------------------------
/* Generation du message etat/alarme général */
void generationMessage() {

  messageId();
  if ( FlagAlarmeTension || FlagLastAlarmeTension || FlagAlarmeIntrusion
       || FlagAlarmeGprs || FlagAlarmeMQTT || FlagAlarmeGps ) {
    message += F("--KO--------KO--");
  }
  else {
    message += F("-------OK-------");
  }
  if (config.Intru && FlagAlarmeIntrusion) {
    message += fl ;
    message += F("-- Intrusion !--") ;			// Intrusion ! ajouter timecomptememo V2-20
    message += fl ;
    message += F("Alarme Motrice");
    message += fl ;
    message += F("1 = ");
    message += FausseAlarme1;
    message += F(", 2 = ");
    message += FausseAlarme2;
    message += fl;
    message += F("Alarme Remorque");
    message += fl ;
    message += F("1 = ");
    message += FausseAlarme3;
    message += F(", 2 = ");
    message += FausseAlarme4;
    //V2-20
    message += fl;
    message += F("Timer (s)= ");
    message += timecomptememo;// /10
    //V2-20
  }
  message += fl;
  if (config.Intru) {
    message += F("Alarme Active ");// ajouter capteur actif futur V2-20
    //V2-20
    if (config.PirActif[0]) {
      message += "1";
    }
    else {
      message += "0";
    }
    if (config.PirActif[1]) {
      message += "1";
    }
    else {
      message += "0";
    }
    if (config.PirActif[2]) {
      message += "1";
    }
    else {
      message += "0";
    }
    if (config.PirActif[3]) {
      message += "1";
    }
    else {
      message += "0";
    }
    //V2-20
    message += fl;
  }
  else {
    message += F("Alarme Arrete");
    message += fl;
  }
  if (config.Silence) {
    message += F("Alarme Silencieuse ON");
    message += fl;
  }
  else
  {
    message += F("Alarme Silencieuse OFF");
    message += fl;
  }
  message += F("Batterie : ");				//"Alarme Batterie : "
  if (FlagAlarmeTension) {
    message += F("Alarme, ");
    // message += fl;// V2-15
  }
  else {
    message += F("OK, ");
    // message += fl;// V2-15
  }
  // V2-15
  message += String(Battpct(TensionBatterie));
  message += "%";
  message += fl;
  // V2-15
  if (config.tracker && !config.Intru) {
    message += F("Gprs ");
    if (FlagAlarmeGprs) {
      message += F("KO");
    } else {
      message += F("OK");
    }
    message += fl;
    message += F("Mqtt ");
    if (FlagAlarmeMQTT) {
      message += F("KO");
    } else {
      message += F("OK");
    }
    message += fl;
    message += F("GPS ");
    if (FlagAlarmeGps) {
      message += F("KO");
    } else {
      message += F("OK");
    }
    message += fl;
  }
}
//---------------------------------------------------------------------------
void sendSMSReply(String num , bool sms) {
  int pseq = 0;
  // si sms=true Envoie SMS, sinon Serialprint seulement
  if (sms) {
    Serial.print(F("SMS Sent "));
    if(message.length() > 150){ // decoupage sms en nseq parties
      int nseq = message.length()/150;
      if(nseq * 150 < message.length()){
        nseq += 1;
      }
      int fin = 0;
      for (byte i = 0; i < nseq;i++){
        if((i+1)*150 > message.length()){
          fin = message.length();
        } else {
          fin = (i+1)*150;
        }
        // Serial.print("seq"),Serial.print(i+1),Serial.print(":"),Serial.print(i*150),Serial.print(":"),Serial.print(fin),Serial.println(":"),Serial.println(message.substring((i*150),fin));
      
        Serial.print(F("sms part :")),Serial.print(i+1),Serial.println(message.substring(0,pseq));
        if (!modem.sendSMS_Multi(num,message.substring((i*150),fin),i+1,nseq)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK"));
        }
        delay(10);
      }
    } else { // 1 seul sms
      if (!modem.sendSMS(num,message)) {
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("OK"));
      }
    }
  }
  Serial.println(F("****************************"));
  Serial.println(message);
  Serial.println(F("****************************"));
}
//---------------------------------------------------------------------------
void read_RSSI() {	// lire valeur RSSI et remplir message
  int r;
  byte n = modem.getSignalQuality();
  // Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  message += F("RSSI=");
  message += String(n);
  message += ", ";
  message += String(r);
  message += F("dBm");
  message += fl;
}
//---------------------------------------------------------------------------
void readmodemtime(){
  String modemHDtate = modem.getGSMDateTime(TinyGSMDateTimeFormat(0));
  // convertir format date time yy/mm/dd,hh:mm:ss
  byte i 	= modemHDtate.indexOf("/");
  byte j 	= modemHDtate.indexOf("/", i + 1);
  N_Y		  = modemHDtate.substring(i - 2, i).toInt();
  N_M 		= modemHDtate.substring(i + 1, j).toInt();
  N_D 		= modemHDtate.substring(j + 1, j + 3).toInt();
  i 	  	= modemHDtate.indexOf(":", 6);
  j     	= modemHDtate.indexOf(":", i + 1);
  N_H 		= modemHDtate.substring(i - 2, i).toInt();
  N_m 		= modemHDtate.substring(i + 1, j).toInt();
  N_S 		= modemHDtate.substring(j + 1, j + 3).toInt();
}
//---------------------------------------------------------------------------
void MajHeure(bool force = false) {  
  Monitoring_Reseau();
  // force = true, force mise à l'heure systeme sur heure modem, meme si defaut NTP
  static bool First = true;
  
  Serial.print(F("Mise a l'heure reguliere !, "));
  Serial.println(First);
  if (First) {															  // premiere fois apres le lancement
    SyncHeureModem(config.hete*4, true);
    readmodemtime();	// lire l'heure du modem
    setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino
    if(!HeureEte()){
      // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hhiver*4));
      SyncHeureModem(config.hhiver*4, true);
      readmodemtime();	// lire l'heure du modem
      setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino
    }
    First = false;
  }
  else {
    //  calcul décalage entre H sys et H reseau en s
    // resynchroniser H modem avec reseau
    if(HeureEte()){
      // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hete*4));
      if(!SyncHeureModem(config.hete*4, false)){
        if(!force)return; // sortie sans mise à l'heure, continue si forcé
      }
    } else {
      // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hhiver*4));
      if(!SyncHeureModem(config.hhiver*4, false)){
        if(!force)return; // sortie sans mise à l'heure, continue si forcé
      }
    }
    readmodemtime();	// lire l'heure du modem
    int ecart = (N_H - hour()) * 3600;
    ecart += (N_m - minute()) * 60;
    ecart += N_S - second();
    Serial.print(F("Ecart s= ")), Serial.println(ecart);

    if (abs(ecart) > 5) {
      Alarm.disable(loopPrincipale);
      // V2-17
      // ArretSonnerie();	// Arret Sonnerie propre correction bug blocagealarme
      ResetSonnerie();
      // V2-17
      Alarm.disable(TSonn);						// les tempos sonnerie sont coupées au cas ou active à ce moment là
      Alarm.disable(TSonnMax);				// mais elles ne sont pas réarmées, elles le seront si nouvelles alarme
      Alarm.disable(TSonnRepos);
      Alarm.disable(MajH);//v1-15
      Alarm.disable(Svie);
      Alarm.disable(HIntruD);//V2-122
      Alarm.disable(HIntruF);//V2-122
      Alarm.disable(Analyse);
      readmodemtime();	// mise à l'heure de l'Arduino
      setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino
      Alarm.write(loopPrincipale, 15);
      Alarm.write(MajH, 3600);
      Alarm.write(Svie, config.Ala_Vie);
      Alarm.write(HIntruD, config.IntruDebut);
      Alarm.write(HIntruF, config.IntruFin);
      Alarm.write(Analyse, 1);
      // if (config.tracker && !config.Intru) {
        // Alarm.write(Send, config.tlent);
      // }
    }
  }
  displayTime(false);
  timesstatus();
  MessageFaussesAlarmes(1);		// V4, V1-15
  AIntru_HeureActuelle(); 		// armement selon l'heure	V2-122
}
//---------------------------------------------------------------------------
void MessageFaussesAlarmes(byte sms) {
  // sms = 1 envoie du sms et RAZ V2-21
  // sms = 0 creation du message sans RAZ, sms sera envoyé par procedure appelante
  // sms = 2 creation du message avec RAZ, sms sera envoyé par procedure appelante
  // V1-12 - V1-13 - V4
  messageId();
  if (FausseAlarme1 > 0 || FausseAlarme2 > 0 || FausseAlarme3 > 0 || FausseAlarme4 > 0) {	// Si fausse alarme envoie sms info nbr fausse alarme
    Serial.print(F("MAJH, Nombre fausse alarmes Motrice  : "));
    Serial.print(FausseAlarme1), Serial.print(F(", ")), Serial.println(FausseAlarme2);
    Serial.print(F("MAJH, Nombre fausse alarmes Remorque : "));
    Serial.print(FausseAlarme3), Serial.print(F(", ")), Serial.println(FausseAlarme4);

    message += F("Fausses Alarmes Motrice");
    message += fl ;
    message += F("1 =");
    message += FausseAlarme1;
    message += F(", 2 =");
    message += FausseAlarme2;
    message += fl;
    message += F("Fausses Alarmes Remorque");
    message += fl ;
    message += F("1 =");				// V2-11
    message += FausseAlarme3;

    message += F(", 2 =");			// V2-11
    message += FausseAlarme4;
    if (sms == 1) {
      if (!FlagAlarmeIntrusion) { // RAZ seulement si pas d'alarme en cours
        FausseAlarme1 = 0;
        FausseAlarme2 = 0;
        FausseAlarme3 = 0;
        FausseAlarme4 = 0;
        CptAlarme1    = 0;
        CptAlarme2    = 0;
        CptAlarme3    = 0;
        CptAlarme4    = 0;
      } else {
        message += fl;
        message += F("pas de RAZ");
      }
      // envoyer à liste preferentielle config.Pos_Pn_PB[x] == 1
      for(int Index = 1; Index <10; Index ++){
        if(config.Pos_Pn_PB[Index] == 1){
          Phone = {"",""};
          if(modem.readPhonebookEntry(&Phone,Index)){
            sendSMSReply(Phone.number, true);
          }
        }
      }
    } else if (sms == 2){ // V4
      FausseAlarme1 = 0;
      FausseAlarme2 = 0;
      FausseAlarme3 = 0;
      FausseAlarme4 = 0;
      CptAlarme1    = 0;
      CptAlarme2    = 0;
      CptAlarme3    = 0;
      CptAlarme4    = 0;
      message += fl;
      message += F("FALARME reset");
    }
  }
  else {
    message += F("Pas de fausses Alarmes");
  }
  // V1-12 - V1-13
}
//---------------------------------------------------------------------------
void ActivationSonnerie() {
  // Sonnerie PN
  if (!SonnMax) {									// pas atteint Temps sonnerie maxi
    if (!config.Silence) digitalWrite(S_Son, HIGH);		// Marche Sonnerie sauf Silence
    Alarm.enable(TSonn);
    if (!FirstSonn) {							// premiere sonnerie on lance Tempo Max
      Alarm.enable(TSonnMax);
      FirstSonn = true;
    }
  }
}
//---------------------------------------------------------------------------
void ArretSonnerie() {
  Serial.print(F("Fin tempo Sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  digitalWrite(S_Son, LOW);	// Arret Sonnerie
  Alarm.disable(TSonn);			// on arrete la tempo sonnerie
  Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
  FirstSonn = false;
  FlagAlarmeIntrusion = false;
  FlagPIR = false;
}
//---------------------------------------------------------------------------
void SonnerieMax() {
  Serial.print(F("Fin periode Max Sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  // fin de la tempo temps de sonnerie maxi
  Alarm.enable(TSonnRepos);	// on lance la tempo repos sonnerie
  Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
  Alarm.disable(TSonn);			// on arrete la tempo sonnerie
  digitalWrite(S_Son, LOW);	// Arret Sonnerie
  FirstSonn = false;///
  SonnMax = true;						// interdit nouveau lancement sonnerie
  // avant fin tempo repos sonnerie
}
//---------------------------------------------------------------------------
void ResetSonnerie() {
  Serial.print(F("Fin periode inhibition sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  // fin de la tempo repos apres temps sonnerie maxi
  Alarm.disable(TSonnRepos);// on arrete la tempo repos
  SonnMax = false;
  ArretSonnerie();
}
//---------------------------------------------------------------------------
void OnceOnly() {
  Serial.println(F("Premier message apres lancement ! "));
  // Serial.println(Alarm.getTriggeredAlarmId());
  if (FlagTempoIntru) {			//	si Alarme Intru était demandée
    config.Intru = true;		//	on reactive
    AllumeCapteur();				// allumage des capteurs selon parametres
    // V1-12 - V1-13 on lance les interruptions
    CptAlarme1 = 0;
    CptAlarme2 = 0;
    CptAlarme3 = 0;
    CptAlarme4 = 0;
    if (config.PirActif[0]) attachInterrupt(digitalPinToInterrupt(Ip_PIR1), IRQ_PIR1, RISING);
    if (config.PirActif[1]) attachInterrupt(digitalPinToInterrupt(Ip_PIR2), IRQ_PIR2, RISING);
    if (config.PirActif[2]) attachInterrupt(digitalPinToInterrupt(Ip_PIR3), IRQ_PIR3, RISING);
    if (config.PirActif[2]) attachInterrupt(digitalPinToInterrupt(Ip_PIR4), IRQ_PIR4, RISING);

  }
  else {
    if (config.tracker) {
      Alarm.write(Send, config.trapide);
      senddata();
    }
  }
  if (config.Pos_PN)envoieGroupeSMS(1,false);	//  envoie message etat apres lancement à liste pref
  if (!config.Intru && config.tracker) {
    Accu = 255;
    Alarm.write(Send, config.trapide);
    Alarm.delay(1000); // attendre envoie sms ci-dessus
    senddata();
  }
  lancement = true;
}
//---------------------------------------------------------------------------
void SignalVie() {
  Serial.print(F("SignalVie "));
  displayTime(false);
  envoieGroupeSMS(0,true);		// envoie groupé
  NbrResetModem = 0;          // reset compteur
  for(byte i = 0; i<5;i++){
    Histo_Reseau[i] = 0; // reset Historique changement reseau
  }
  modem.deleteSmsMessage(0,4);// au cas ou, efface tous les SMS envoyé/reçu
}
//---------------------------------------------------------------------------
void sauvConfig() {
  // Sauvegarde config en EEPROM
  Serial.print(F("long EEPROM:"));
  Serial.println(EEPROM_writeAnything(EEPROM_adresse[2], config));		//	ecriture EEPROM
  Alarm.delay(100);
  EEPROM_readAnything(EEPROM_adresse[2], config);
  Alarm.delay(500);	// V2-19
}
//---------------------------------------------------------------------------
void displayTime(bool m) {
  // m = true ajouter Time à message
  String dt;
  if (day() < 10) {
    dt += "0";
  }
  dt += day();
  dt += ("/");
  if (month() < 10) {
    dt += "0";
  }
  dt += month();
  dt += ("/");
  dt += year();
  dt += (" ");
  if (hour() < 10) {
    dt += "0";
  }
  dt += hour();
  dt += ":";
  if (minute() < 10) {
    dt += "0";
  }
  dt += minute();
  dt += ":";
  if (second() < 10) {
    dt += "0";
  }
  dt += second();
  if (m) message += dt;
  Serial.println(dt);
}
//---------------------------------------------------------------------------
void flushSerial() {
  while (Serial.available())
    Serial.read();
}
//---------------------------------------------------------------------------
void timesstatus() {	// etat synchronisation time/heure systeme
  Serial.print(F("Synchro Time  : "));
  switch (timeStatus()) {
    case 0:
      Serial.println(F(" pas synchro"));
      break;
    case 1:
      Serial.println(F(" defaut synchro"));
      break;
    case 2:
      Serial.println(F(" OK"));
      break;
  }
}
//---------------------------------------------------------------------------
void softReset() {
  //asm volatile ("  jmp 0");	//	Reset Soft
  wdt_enable(WDTO_250MS);					// activation du watchdog

  while (1);

}
//---------------------------------------------------------------------------
int freeRam () {	// lecture RAM free
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
//---------------------------------------------------------------------------
bool HeureEte() {
  // Serial.print("mois:"),Serial.print(month());
  // Serial.print(" jour:"),Serial.print(day());
  // Serial.print(" jsem:"),Serial.println(weekday());
  // return true en été, false en hiver (1=dimanche)
  bool Hete = false;
  if (month() > 10 || month() < 3
      || (month() == 10 && (day() - weekday()) > 22)
      || (month() == 3  && (day() - weekday()) < 24)) {
    Hete = false;                      								// c'est l'hiver
  }
  else {
    Hete = true;                       								// c'est l'été
  }
  // if(Hete == true){
  //   Serial.println("Heure été");
  // } else {Serial.println("Heure hiver");}
  return Hete;
}
//---------------------------------------------------------------------------
bool decodeGPS() {
  float alt = 0;
  int vsat, usat;
  static float lastheading = 0;
  bool fix = modem.getGPS(&lat, &lon, &speed, &alt, &heading, &vsat, &usat);
  if (!fix){
    AlarmeGps = true;
    modem.disableGPS();
    delay(500);
    modem.enableGPS(); // relance Allumage GPS
    return fix;        // sortir si fix false
  }

  if (speed < config.vtransition){ // evité course fantaisiste à l'arret, recopie course precedent
    heading = lastheading;
  }
  lastheading = heading;

  if(flagtestspeed)speed=10.0; // pour test seulement

  AlarmeGps = false;
  return fix;
}
//---------------------------------------------------------------------------
void logRecord(String nom, String action) { // renseigne log et enregistre EEPROM
  static int index = 0;
  String temp;
  if (month() < 10) {
    temp =  "0" + String(month());
  }
  else {
    // temp =  "0";	//V2-18
    temp = String(month());	//V2-18
  }
  if (day() < 10 ) {
    temp += "0" + String(day());
  }
  else {
    temp += String(day());
  }
  if (hour() < 10) {
    temp += "-0" + String(hour());
  }
  else {
    temp += "-" + String(hour());
  }
  if (minute() < 10) {
    temp += "0" + String(minute());
  }
  else {
    temp += String(minute());
  }
  temp  .toCharArray(record[index].dt, 10);
  nom   .toCharArray(record[index].Name, 15);
  action.toCharArray(record[index].Act, 2);

  int longueur = EEPROM_writeAnything(EEPROM_adresse[1], record);// enregistre en EEPROM

  if (index < 4) {
    index ++;
  }
  else {
    index = 0;
  }
}
//---------------------------------------------------------------------------
void AllumeCapteur() {		// allumage des capteurs selon parametres

  // V2-18
  for (int i = 0; i < 10; i++) {
    if (config.PirActif[0]) {
      digitalWrite(Op_PIR1, HIGH);
    }
    if (config.PirActif[1]) {
      digitalWrite(Op_PIR2, HIGH);
    }
    if (config.PirActif[2]) {
      digitalWrite(Op_PIR3, HIGH);
    }
    if (config.PirActif[3]) {
      digitalWrite(Op_PIR4, HIGH);
    }
    Alarm.delay(400);
    if (config.PirActif[0]) {
      digitalWrite(Op_PIR1, LOW);
    }
    if (config.PirActif[1]) {
      digitalWrite(Op_PIR2, LOW);
    }
    if (config.PirActif[2]) {
      digitalWrite(Op_PIR3, LOW);
    }
    if (config.PirActif[3]) {
      digitalWrite(Op_PIR4, LOW);
    }
    if ((i % 5) == 0) {
      Alarm.delay(500);
    }
    else {
      Alarm.delay(100);
    }
  }
  if (config.PirActif[0]) {
    digitalWrite(Op_PIR1, HIGH);
  }
  if (config.PirActif[1]) {
    digitalWrite(Op_PIR2, HIGH);
  }
  if (config.PirActif[2]) {
    digitalWrite(Op_PIR3, HIGH);
  }
  if (config.PirActif[3]) {
    digitalWrite(Op_PIR4, HIGH);
  }

  if (!config.PirActif[0]) {
    digitalWrite(Op_PIR1, LOW);
  }
  if (!config.PirActif[1]) {
    digitalWrite(Op_PIR2, LOW);
  }
  if (!config.PirActif[2]) {
    digitalWrite(Op_PIR3, LOW);
  }
  if (!config.PirActif[3]) {
    digitalWrite(Op_PIR4, LOW);
  }

  Alarm.delay(500);					 // on attend stabilisation des capteurs
}
//--------------------------------------------------------------------------------//
int moyenneAnalogique() {	// calcul moyenne 10 mesures consécutives
  int moyenne = 0;
  for (int j = 0; j < 10; j++) {
    delay(10);
    moyenne += analogRead(Ip_Analogique);
  }
  moyenne /= 10;
  return moyenne;
}
//---------------------------------------------------------------------------
void AIntru_HeureActuelle() { //	V2-122
  // chagement parametre intru en fonction de l'heure actuelle
  long Heureactuelle = hour() * 60; // calcul en 4 lignes sinon bug!
  Heureactuelle += minute();
  Heureactuelle  = Heureactuelle * 60;
  Heureactuelle += second(); // en secondes

  if (config.IntruDebut > config.IntruFin) {
    if ((Heureactuelle > config.IntruDebut && Heureactuelle > config.IntruFin)
        || (Heureactuelle < config.IntruDebut && Heureactuelle < config.IntruFin)) {
      // Nuit
      IntruD();
    }
    else {	// Jour
      IntruF();
    }
  }
  else {
    if (Heureactuelle > config.IntruDebut && Heureactuelle < config.IntruFin) {
      // Nuit
      IntruD();
    }
    else {	// Jour
      IntruF();
    }
  }
  // Serial.print(F("Hintru = ")),Serial.print(Heureactuelle),Serial.print(",");
  // Serial.print(config.IntruDebut),Serial.print(","),Serial.print(config.IntruFin);
  // Serial.print(","),Serial.println(config.Intru);
}
//---------------------------------------------------------------------------
void IntruF() { // Charge parametre Alarme Intrusion Jour	//	V2-122
  Nmax 			= config.Jour_Nmax;
  TmCptMax  = config.Jour_TmCptMax;
}
//---------------------------------------------------------------------------
void IntruD() { // Charge parametre Alarme Intrusion Nuit	//	V2-122
  Nmax 			= config.Nuit_Nmax;
  TmCptMax  = config.Nuit_TmCptMax;
}
//---------------------------------------------------------------------------
void PrintEEPROM() {
  Serial.print(F("config.magic = "))				, Serial.println(config.magic);
  Serial.print(F("config.Ala_Vie = "))			, Serial.println(config.Ala_Vie);
  Serial.print(F("config.Intru = "))				, Serial.println(config.Intru);
  Serial.print(F("config.Silence = "))			, Serial.println(config.Silence);
  Serial.print(F("config.Pos_PN = "))				, Serial.println(config.Pos_PN);
  Serial.print(F("config.Dsonn = "))				, Serial.println(config.Dsonn);
  Serial.print(F("config.DsonnMax = "))			, Serial.println(config.DsonnMax);
  Serial.print(F("config.Dsonnrepos = "))		, Serial.println(config.Dsonnrepos);
  Serial.print(F("config.Jour_TmCptMax = ")), Serial.println(config.Jour_TmCptMax);
  Serial.print(F("config.Jour_Nmax = "))		, Serial.println(config.Jour_Nmax);
  Serial.print(F("config.PirActif[0] = "))	, Serial.println(config.PirActif[0]);
  Serial.print(F("config.PirActif[1] = "))	, Serial.println(config.PirActif[1]);
  Serial.print(F("config.PirActif[2] = "))	, Serial.println(config.PirActif[2]);
  Serial.print(F("config.PirActif[3] = "))	, Serial.println(config.PirActif[3]);
  Serial.print(F("config.Idchar = "))				, Serial.println(config.Idchar);
  Serial.print(F("config.IntruAuto = "))		, Serial.println(config.IntruAuto);
  Serial.print(F("config.IntruFin	= "))			, Serial.println(config.IntruFin);
  Serial.print(F("config.IntruDebut = "))		, Serial.println(config.IntruDebut);
  Serial.print(F("config.Nuit_TmCptMax = ")), Serial.println(config.Nuit_TmCptMax);
  Serial.print(F("config.Nuit_Nmax = "))		, Serial.println(config.Nuit_Nmax);
  Serial.print(F("CoeffTension = "))	      , Serial.println(CoeffTension);
  Serial.print(F("apn = "))                 , Serial.println(config.apn);
  Serial.print(F("gprsUser = "))            , Serial.println(config.gprsUser);
  Serial.print(F("gprspass = "))            , Serial.println(config.gprsPass);
  Serial.print(F("mqttServeur = "))         , Serial.println(config.mqttServer);
  Serial.print(F("mqttPort = "))            , Serial.println(config.mqttPort);
  Serial.print(F("mqttUserName = "))        , Serial.println(config.mqttUserName);
  Serial.print(F("mqttPass = "))            , Serial.println(config.mqttPass);
  Serial.print(F("writeTopic = "))          , Serial.println(config.writeTopic);
  Serial.print(F("Tempo rapide = "))        , Serial.println(config.trapide);
  Serial.print(F("Tempo lente = "))         , Serial.println(config.tlent);
  Serial.print(F("Vitesse mini = "))        , Serial.println(config.vtransition);
  Serial.print(F("Compt ala tracker = "))   , Serial.println(config.cptAla);
  Serial.print(F("Pos_Pn_PB = "));
  for (int i = 1; i < 10; i++) {
    Serial.print(config.Pos_Pn_PB[i]);
    if (i == 9) {
      Serial.println();
    }
    else {
      Serial.print(F(","));
    }
  }

  for (int i = 0; i < 5; i++) {
    Serial.print(F("log ")), Serial.print(i), Serial.print(F(" = "));;
    Serial.print(record[i].dt);
    Serial.print(F(","));
    Serial.print(record[i].Act);
    Serial.print(F(","));
    Serial.println(record[i].Name);
  }
}

//---------------------------------------------------------------------------
int Battpct(long vbat) {
  //V2-15 calcul Etat batterie en %
  //V1-16 correction bug <= certaines lignes, remplacé par > partout
  int EtatBat = 0;
  if (vbat > 1260) {
    EtatBat = 100;
  }
  else if (vbat > 1255) {
    EtatBat = 95;
  }
  else if (vbat > 1250) {
    EtatBat = 90;
  }
  else if (vbat > 1246) {
    EtatBat = 85;
  }
  else if (vbat > 1242) {
    EtatBat = 80;
  }
  else if (vbat > 1237) {
    EtatBat = 75;
  }
  else if (vbat > 1232) {
    EtatBat = 70;
  }
  else if (vbat > 1226) {
    EtatBat = 65;
  }
  else if (vbat > 1220) {
    EtatBat = 60;
  }
  else if (vbat > 1213) {
    EtatBat = 55;
  }
  else if (vbat > 1206) {
    EtatBat = 50;
  }
  else if (vbat > 1198) {
    EtatBat = 45;
  }
  else if (vbat > 1190) {
    EtatBat = 40;
  }
  else if (vbat > 1183) {
    EtatBat = 35;
  }
  else if (vbat > 1175) {
    EtatBat = 30;
  }
  else if (vbat > 1167) {
    EtatBat = 25;
  }
  else if (vbat > 1158) {
    EtatBat = 20;
  }
  else if (vbat > 1145) {
    EtatBat = 15;
  }
  else if (vbat > 1131) {
    EtatBat = 10;
  }
  else if (vbat > 1100) {
    EtatBat = 5;
  }
  else if (vbat <= 1050) {
    EtatBat = 0;
  }
  return EtatBat;
}
//--------------------------------------------------------------------------------//
void messageId() { // V2-21
  message = Id;
  displayTime(true); // V2-21
  message += fl;     // V2-21
}
//--------------------------------------------------------------------------------//
void gereVoyant() {
  if ( digitalRead(Ip_PIR1)) {
    digitalWrite(led_PIR1, HIGH);	// allume led locale temoin alarme
  }
  else {
    digitalWrite(led_PIR1, LOW);
  }
  if ( digitalRead(Ip_PIR2)) {
    digitalWrite(led_PIR2, HIGH);	// allume led locale
  }
  else {
    digitalWrite(led_PIR2, LOW);
  }
  if ( digitalRead(Ip_PIR3)) {
    digitalWrite(led_PIR3, HIGH);	// allume led locale
  }
  else {
    digitalWrite(led_PIR3, LOW);
  }
  if ( digitalRead(Ip_PIR4)) {
    digitalWrite(led_PIR4, HIGH);	// allume led locale
  }
  else {
    digitalWrite(led_PIR4, LOW);
  }
}
//---------------------------------------------------------------------------
void senddata() {
  if(decodeGPS()){									// Lire GPS
    char bidon[20];
    sprintf(bidon, "%04d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
    // {"Id":"BB63000","date_tx_sms":"2020-02-10 15:15:15","lat":42.123456,"lon":2.123456,"speed":25.2,"course":180}

    DynamicJsonDocument JsonDoc (164);
    JsonDoc["Id"]          = config.Idchar;
    JsonDoc["date_tx_sms"] = bidon;
    JsonDoc["lat"]         = String(lat,6);
    JsonDoc["lon"]         = String(lon,6);
    JsonDoc["speed"]       = String(speed, 1);
    JsonDoc["course"]      = String(heading, 0);

    Serial.print(F("MQTT connecté:")),Serial.print(mqtt.connected());
    Serial.print(F(", GPRS connecté :")),Serial.println(modem.isGprsConnected());

    char JSONmessageBuffer[200];
    serializeJson(JsonDoc, JSONmessageBuffer);
    serializeJson(JsonDoc, Serial);
    if(modem.isNetworkConnected() && modem.isGprsConnected() && mqtt.connected()){
      if (! mqtt.publish(config.writeTopic,JSONmessageBuffer)) {// envoie data MQTT
        Serial.println(F("mqtt connect Failed"));
      } else {
        Serial.println(F("mqtt connected"));
        AlarmeMQTT = false;
        AlarmeGprs = false;
      }
    }
  }
}
//---------------------------------------------------------------------------
boolean mqttConnect() {
  Serial.print(F("Connecting to "));
  Serial.print(config.mqttServer);

  // Connect to MQTT Broker
  // boolean status = mqtt.connect("GsmClientTest");
  bool status = mqtt.connect(config.Idchar, config.mqttUserName, config.mqttPass);

  // Or, if you want to authenticate MQTT:
  // boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

  if (status == false) {
    Serial.println(F(" fail"));
    return false;
  }
  Serial.println(F(" success"));
  // mqtt.publish(config.writeTopic, "GsmClientTest started");
  // mqtt.subscribe(topicLed);
  // mqtt.subscribe(topicnetwork);
  return mqtt.connected();
}

//---------------------------------------------------------------------------
void gereCadence() {
  // change cadence envoie GPS data selon la vitesse
  static bool lastroule = false;
  if (int(speed) > config.vtransition) {
    if (Accu < 255 - int(speed)) {
      Accu += speed;
    }
  } else {
    if (Accu >= 10) {
      Accu = Accu - 10;
      if (Accu < 10)Accu = 0;
    }
  }
  if(speed == 0 && Accu < 10) Accu = 0;
  if (Accu == 0 && lastroule == true) {
    lastroule = false;
    Alarm.disable(Send);
    Alarm.write(Send, config.tlent);
    senddata();
    Serial.print(F("cadence  = ")), Serial.print(config.tlent);
    Serial.print(F(", setalarme  = ")), Serial.println(Alarm.read(Send));
  }
  else if (Accu > 0 && lastroule == false) {
    lastroule = true;
    Alarm.disable(Send);
    Alarm.write(Send, config.trapide);
    senddata();
    Serial.print(F("cadence  = ")), Serial.print(config.trapide);
    Serial.print(F(", setalarme  = ")), Serial.println(Alarm.read(Send));
  }
  Serial.print(F("gerecadence.speed = ")), Serial.print(speed);
  Serial.print(F(", Accu.speed  = ")), Serial.println(Accu);
}
//---------------------------------------------------------------------------
void ArretLocalisation() {
  // fin de gelocalisation
  Alarm.disable(Send);
  mqtt.disconnect();

  FlagAlarmeGprs = false;
  FlagAlarmeMQTT = false;
  FlagAlarmeGps  = false;
}
//---------------------------------------------------------------------------
bool Cherche_N_PB(String numero){ // Cherche numero dans PB
  for (byte idx = 1; idx < 10; idx++){
    Phone = {"",""};
    if(modem.readPhonebookEntry(&Phone, idx)){
      if(Phone.number == numero){
        Serial.print(F("Nom :")), Serial.println(Phone.text);
        return true;
      }
    }// else { idx = 10;}    
  }
  return false;
}
//---------------------------------------------------------------------------
bool SyncHeureModem(int Savetime, bool FirstTime){
  int rep = 1;
  int compteur = 0;
  if(modem.isNetworkConnected() && modem.isGprsConnected()){
    do{
      rep = modem.NTPServerSync(NTPServer, Savetime);
      Serial.print("NTP:"),Serial.println(modem.ShowNTPError(rep));
      if (compteur > 10){ // 10 tentatives
        if(FirstTime){
          // echec Synchro, force date 01/08/2022 08:00:00, jour toujours circulé
          modem.send_AT("+CCLK=\"22/08/01,08:00:00+08\"");
          delay(500);
        }
        return false;
      }
      compteur += 1;
      delay(1000);
    } while(rep != 0);
    return true;
  }
  return false;
}
//---------------------------------------------------------------------------
String ConnectedNetwork(){  
  String network = "";
  int n = modem.getNetworkCurrentMode();
  if(n==0){ network            = F("not connect");}
  else if(n>0 && n<4){network  = F("2G");}
  else if(n>=4 && n<8){network = F("3G");}
  else if(n==8){network        = F("4G");}
  else if(n==16){network       = F("XLTE:CDMA&LTE");}
  else {network = String(n);}
  return network;
}
//---------------------------------------------------------------------------
/* Enregistre mode cnx Reseau
  Histo-Reseau(0) = not connect, (1)=inconnu, (2)=2G, (3)=3G, (4)=4G 
  cummul à chaque heure */
void Monitoring_Reseau(){
  int n = modem.getNetworkCurrentMode();
  if(n==0){ Histo_Reseau[0]            ++ ;}
  else if(n>0 && n<4){Histo_Reseau[2]  ++;}
  else if(n>=4 && n<8){Histo_Reseau[3] ++;}
  else if(n==8){Histo_Reseau[4]        ++;}
  else {Histo_Reseau[1] ++;}
}
//---------------------------------------------------------------------------
void message_Monitoring_Reseau(){
  message += F("Histo Network Chgt : ");
  message += fl;
  message += F("not connect : ");
  message += String(Histo_Reseau[0]);
  message += fl;
  message += F("inconnu : ");
  message += String(Histo_Reseau[1]);
  message += fl;
  message += F("2G : ");
  message += String(Histo_Reseau[2]);
  message += fl;
  message += F("3G : ");
  message += String(Histo_Reseau[3]);
  message += fl;
  message += F("4G : ");
  message += String(Histo_Reseau[4]);
  message += fl;
}
/* --------------------  test seulement ----------------------*/
void recvOneChar() {

  char   receivedChar;
  static String serialmessage = "";
  static bool   newData = false;

  if (Serial.available() > 0) {
    receivedChar = Serial.read();
    if (receivedChar != 10 && receivedChar != 13) {
      serialmessage += receivedChar;
    }
    else {
      newData = true;
      return;
    }
  }
  if (newData == true) {
    Serial.println(serialmessage);
    interpretemessage(serialmessage);
    newData = false;
    serialmessage = "";
  }
}
void interpretemessage(String demande) {
  String bidons;
  // demande.toUpperCase();
  if (demande.indexOf(char(61)) == 0) { // "="
    bidons = demande.substring(1); //(demande.indexOf("=")+1);
    // Serial.print(bidons),Serial.println(bidons.length());
    // bidons.toCharArray(replybuffer, bidons.length() + 1);
    // traite_sms(99);	//	traitement SMS en mode test local
    message = bidons;
    FlagMessageLocal = true; //	traitement SMS en mode test local
  }
}
