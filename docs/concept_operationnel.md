# Concept opérationnel — Quad4D_rebooted en volière


## 1. Contexte et mission

Quad4D_rebooted fait voler plusieurs drones en même temps dans la volière de l'ENAC, pour
des shows chorégraphiés en intérieur. Les drones tournent sous Paparazzi et ne se
localisent pas par GPS : leur position vient du système de capture de mouvement
OptiTrack, injectée dans chaque drone par la liaison de données.

L'IHM Quad4D-rebooted (Click'n Fly) est l'outil de l'opérateur pendant le show : elle
permet de choisir un scénario prédéfini ou d'en composer un à partir de la
bibliothèque de trajectoires, de suivre les drones en 3D et sur la télémétrie
live, et de conduire l'ensemble du vol.

En pratique, une session se fait avec un seul opérateur, qui a donc les mains
sur le clavier et les yeux qui alternent entre l'écran et la volière. Ce double
rôle est structurant : tout ce que le système lui demande de faire pendant le
vol doit être faisable sans quitter des yeux ce qui se passe derrière le filet.

## 2. Séquence nominale

La conduite du vol tient maintenant dans l'IHM. Le Paparazzi Center sert
encore à lancer la session, et le GCS reste ouvert en secours, mais l'opérateur
n'a plus à y revenir en fonctionnement normal.

| # | Étape | Outil |
|---|---|---|
| 1 | Placer les drones, batteries branchées, dans la volière | — (physique) |
| 2 | Allumer une radiocommande par drone | — (physique) |
| 3 | Build des drones qui vont voler, lancement des opérations | Paparazzi Center |
| 4 | Lancer l'IHM (double-clic sur l'icône) | IHM Quad4D |
| 5 | Choisir ou composer le scénario | IHM Quad4D |
| 6 | **Lancer l'analyse** : conflits détectés puis résolus, avant tout décollage | IHM Quad4D |
| 7 | Vérifier la checklist par drone : pose mocap, RC, liaison, batterie | IHM Quad4D |
| 8 | **Décoller** : moteurs, décollage et mise en place aux points de standby | IHM Quad4D |
| 9 | Lancer le show | IHM Quad4D |
| 10 | **Stop** en fin de show : retour aux points de standby | IHM Quad4D |
| 11 | **Land all**, puis coupure des moteurs | IHM Quad4D |

L'ordre compte : **l'analyse se fait au sol, avant le décollage.** Les
trajectoires sont rejouées hors ligne, les conflits détectés, puis résolus par
ordonnancement. Rien ne décolle tant que le scénario n'a pas été déclaré sûr.

Un unique bouton **Décoller** enchaîne le démarrage des moteurs, le décollage et
la mise en place. **Stop** ramène les drones aux points de standby, à tout
moment du show. **Land all** les fait atterrir, et **Kill**, par drone et avec
confirmation à deux clics, reste visible en permanence.

Entre les points de standby et les points de départ du scénario, l'application
gère elle-même le **transit** : elle choisit sans rien demander à l'opérateur la
méthode la plus simple qui soit sûre pour la géométrie du moment — séquencement
(un drone à la fois, le plus agréable à regarder), sinon départs échelonnés,
sinon étagement en hauteur, qui reste sûr quelle que soit la configuration. Le
même mécanisme joue au retour, y compris quand le Stop survient à un instant
quelconque du show.

Pendant le show, les conflits entre trajectoires sont résolus par
**ordonnancement sur le chemin** : un drone moins prioritaire patiente sur sa
propre trajectoire jusqu'à ce que la voie soit libre. La géométrie des figures
n'est jamais déformée.

À côté de cette séquence, deux réglages sont *persistants* : ils ne se refont
pas à chaque session, seulement quand quelqu'un a touché à la configuration.

- **Le fichier de télémétrie de chaque drone doit être `vto_wfb.xml`.** Le
  fichier générique `default_rotorcraft` est trop bavard : il sature la bande
  passante radio et étouffe l'uplink `EXTERNAL_POSE` qui porte la position
  OptiTrack — le drone dérive en vol alors que tout semble normal au sol.
- **Après un changement de configuration, il faut reflasher le drone**, sinon
  les settings embarqués ne correspondent plus à ceux que le sol croit envoyer.

## 3. Exceptions et conduite à tenir

Ce qui suit vient du vécu des sessions de juillet et août 2026 : ce sont les
problèmes réellement rencontrés, plus deux cas jamais observés mais suffisamment
critiques pour être anticipés.

| Exception | Comment on la détecte | Réponse |
|---|---|---|
| Uplink saturé (mauvais fichier de télémétrie) | Dérive du drone en vol, l'affichage sol reste normal | Passer la télémétrie en `vto_wfb.xml` — à vérifier *avant* de voler |
| Settings désynchronisés après un changement de conf | Erreur « No settings #… » dans le log pprz | Reflasher le drone |
| Passage en Guided refusé | Le drone reste en NAV au lancement | Firmware inadapté : reprogrammer l'autopilote |
| Pose OptiTrack absente au sol | Icône rouge dans la checklist de l'IHM | Vérifier le réseau du PC et le process NatNet |
| Un drone n'atteint pas son point de départ | L'IHM le nomme et affiche la distance restante | Au bout de 15 s, elle propose de lancer quand même ; un drone qui ne se stabilise jamais a en général un défaut matériel |
| Batterie basse au lancement | Confirmation demandée, avec la tension par cellule | Décision de l'opérateur : changer le pack, ou voler en connaissance de cause |
| Batterie critique en vol | Atterrissage automatique de toute la flotte | Aucune action requise, l'IHM déclenche le land all |
| Perte OptiTrack en plein vol | *(jamais observée)* — l'affichage se fige, il ne bascule pas sur la télémétrie | Faire atterrir tout le monde immédiatement. **L'IHM ne signale rien** — voir §5 |
| Un ou deux drones deviennent erratiques | À l'œil, en regardant la volière | Land all depuis l'IHM ; kill par drone en dernier recours |

**Land plutôt que kill.** Quand un drone déraille, deux réponses existent :
l'atterrissage commandé et le kill (coupure moteurs immédiate, le drone tombe).
Le kill arrête le danger plus vite, mais l'impact casse du matériel. La règle
est donc de faire atterrir par défaut, et de réserver le kill aux cas où le
drone met en danger quelque chose de plus précieux que lui-même — typiquement
s'il fonce vers le filet, un autre drone, ou une personne.

**Quand un drone doit être posé, tout le monde se pose.** Sans évitement
réactif, rien ne garantit que la descente d'un drone ne croise pas la
trajectoire d'un autre. Le land sélectif, poser le fautif pendant que le show
continue, reste une perspective, qui ne sera envisageable qu'avec une garantie
de non-croisement des descentes.

## 4. Perspectives

Ce qui n'est pas fait, par ordre d'intérêt décroissant.

- **Retomber sur la télémétrie quand la mocap décroche** (voir §5, c'est
  aujourd'hui la limite la plus gênante).
- **Indicateur de qualité du lien datalink**, et chronomètre de vol par drone.
- **Autonomie restante affichée en information**, sans blocage.
- **Land sélectif**, subordonné à une garantie de non-croisement des descentes.
- **Son pendant les scénarios.**

## 5. Limites et hypothèses

- **Deux sources de position, mais une bascule définitive.** L'application sait
  se positionner de deux façons : la mocap (`EXTERNAL_POSE`) et l'estimation de
  l'autopilote lui-même (`ROTORCRAFT_FP`). Un drone qui n'a jamais reçu de pose
  OptiTrack vole sur sa télémétrie; c'est ce qui permet de faire voler un drone
  simulé, seul ou aux côtés d'un drone réel. Mais dès qu'un drone a reçu **une
  seule** position OptiTrack, il ignore sa télémétrie pour le reste de la
  session. Si la mocap décroche en vol, **la position affichée se fige
  silencieusement** sur le dernier échantillon : l'écran montre un drone
  immobile, sans alarme, alors que la télémétrie continue d'arriver. C'est la
  limite la plus sérieuse du système actuel, et le premier chantier à reprendre.
- **Règle batterie.** Les seuils ne sont pas dans le code : ils sont lus dans la
  section `BAT` du fichier `airframe` de chaque drone, celui-là même qu'utilise
  l'autopilote, l'IHM et le drone ne peuvent donc pas diverger sur ce qu'est
  une batterie basse. L'affichage se fait **par cellule**, ce qui reste lisible
  quel que soit le nombre d'éléments du pack. Sous le seuil bas, l'IHM **demande
  confirmation** plutôt que d'interdire : refuser un vol d'une minute pour
  changer un pack coûtait plus qu'il ne rapportait. Sous le seuil critique, le
  lancement est refusé sans appel, et un franchissement en vol déclenche un land
  all automatique.
- **Faisabilité dynamique.** La vitesse de pointe de chaque trajectoire est
  plafonnée à 1,5 m/s, en ralentissant celles qui dépassent et en laissant les
  autres intactes. Un curseur permet en outre de moduler la vitesse du show
  entre ×0,5 et ×1,5, y compris en vol : tous les drones sont affectés du même
  facteur, ce qui préserve leur synchronisation et la validité de la
  déconfliction.
- **Pas d'évitement réactif.** Un évitement par champ de potentiel a été
  développé et essayé en simulation, sur le show comme sur les transits. Il
  produisait des rapprochements dangereux et des blocages, et il a été
  **écarté**. La sécurité repose sur des méthodes
  calculées avant le vol, prévisibles et validables en simulation.
- **La déconfliction sur le chemin a une limite connue.** Quand un drone doit
  stationner à un endroit que traverse un autre, aucune loi horaire ne les
  sépare; c'est pourquoi les transits disposent aussi de l'étagement en
  hauteur, qui agit sur la géométrie.
- **Une radiocommande par drone** : l'armement Paparazzi exige un lien RC actif ;
  sans lui, le passage en Guided est refusé silencieusement.
- **Un show sérieux dépend de l'OptiTrack.** La télémétrie de l'autopilote reste
  un repli utilisable, et c'est elle qui porte la simulation, mais en intérieur
  et sans GPS son estimation dérive : elle ne permet pas la précision qu'exige
  une chorégraphie à plusieurs drones dans un espace confiné.
- **Le GCS reste le secours.** L'IHM a repris les commandes courantes mais n'a
  rien supprimé : en cas de doute ou de panne, le GCS garde tous ses moyens.

