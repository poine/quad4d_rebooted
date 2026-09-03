# Note de passation — Quad4D_rebooted / Click'n Fly

*Dieynaba DIOP — septembre 2026, fin de stage*

Ce document dit ce qui n'est pas fait, ce qui est fragile, et ce qu'il faut
savoir avant de toucher au code. Il est écrit pour la personne qui reprendra le
projet, pas pour juger le travail : la description de ce qui marche est dans le
[README](../README.md) et le [concept opérationnel](concept_operationnel.md).

## Les trois choses à lire avant tout

### 1. La position se fige quand la mocap décroche

L'application dispose de deux sources de position : la capture de mouvement
(`EXTERNAL_POSE`) et l'estimation de l'autopilote (`ROTORCRAFT_FP`). Un drone
qui n'a jamais reçu de pose OptiTrack vole sur sa télémétrie — c'est ce qui
permet la simulation, seule ou mêlée à un drone réel.

Mais la bascule est **définitive** : dès qu'un drone a reçu une seule position
OptiTrack, `on_pprz_flight_param` ignore sa télémétrie pour le reste de la
session. Si la mocap décroche en vol, la position affichée reste bloquée sur le
dernier échantillon, **sans aucune alarme**, pendant que la télémétrie continue
d'arriver, inutilisée.

Le correctif est court et connu : préférer la mocap **tant qu'elle est fraîche**
plutôt que pour toujours, en comparant `t_last_ext_pose` à l'instant courant.
Il n'a pas été fait faute de temps de vol pour le valider. C'est le premier
chantier à reprendre.

### 2. Le HOOPS_110 se comporte mal par moments, cause non établie

**L'observation, d'abord.** Le 110 vole normalement une fois sur deux. Il est
correctement repéré par la mocap, tout paraît sain au sol, puis il part sans
raison identifiée. Le caractère intermittent est le point important : je n'ai
pas trouvé de déclencheur reproductible.

**Une piste, trouvée en comparant les fichiers de configuration.** Entre son
`airframe` et celui du 112, trois différences ressortent :

- son `ACCEL_CALIB` n'a ni `rotation` ni `TAWAKIV2_IMU_ROT`, que son
  `GYRO_CALIB` possède — accéléromètre et gyromètre ne seraient donc pas
  exprimés dans le même repère ;
- son `MAG_CALIB` a le même manque ;
- un `ACCEL_AAF` supplémentaire y est déclaré, absent du 112, qui ajoute du
  retard de phase sur la mesure d'accélération — ce à quoi l'INDI est sensible.

**Ce qui n'est pas démontré.** Un repère faux serait systématique, alors que le
défaut ne l'est pas. Il reste compatible avec l'observation si son effet est
faible à faible inclinaison et grandit dans les manœuvres, mais **ce n'est pas
vérifié**. Et d'autres causes n'ont pas été écartées : vibrations ou hélice mal
équilibrée, moteur ou ESC, et la saturation de la liaison, qui provoque elle
aussi de la dérive quand la télémétrie n'est pas la bonne.

Bref, une hypothèse cohérente, pas un diagnostic.

**Un point à savoir dans tous les cas** : la cible `nps` des deux airframes
définit `TAWAKIV2_IMU_ROT` à la chaîne vide. Si le repère est en cause, il est
**invisible en simulation**, ce qui explique qu'on puisse chercher longtemps du
mauvais côté.

**Le test préparé, non exécuté.** Ajouter `rotation` et `TAWAKIV2_IMU_ROT` à
l'`ACCEL_CALIB` du 110 sur une *copie* de l'airframe, en gardant ses valeurs de
neutre et d'échelle — elles restent valables, s'appliquant axe par axe dans le
repère capteur, la rotation venant après. Puis, au sol, incliner le drone et
vérifier que l'attitude affichée penche du même côté et du bon axe, en
comparant au 112 dans les mêmes positions. Rien n'a été appliqué : l'opérateur
devait d'abord recalibrer les drones.

### 3. L'évitement réactif a été essayé et écarté

Un évitement par champ de potentiel a été développé et réglé, puis essayé en
simulation sur le show et sur les transits. Dans les deux cas il produisait des
**rapprochements dangereux et des blocages**. 

## Ce que la déconfliction garantit, et ce qu'elle ne garantit pas

**Pendant le show**, les conflits sont résolus par ordonnancement sur le chemin :
un drone moins prioritaire patiente sur sa propre trajectoire. La géométrie
n'est jamais déformée.

Deux réserves:

- la priorité est l'ordre du scénario, sans autre critère ;
- les conflits sont résolus **sur le premier cycle du show**. Comme les
  attentes allongent les durées, les cycles suivants peuvent dériver vers des
  déphasages non analysés. La courbe de distance minimale entre drones, dans la
  télémétrie live, est là pour ça : elle se surveille.

**Pendant les transits**, l'application choisit seule entre séquencement,
départs échelonnés et étagement en hauteur. Ce dernier reste sûr quelle que soit
la géométrie, d'où son rôle de repli.

La limite de fond de l'ordonnancement : quand un drone doit **stationner** à un
endroit que traverse un autre, aucune loi horaire ne les sépare. Seule la
géométrie peut résoudre ce cas, et c'est pourquoi l'étagement existe.

## Non validé en vol

- **Le forçage de départ** (`transit_shortfalls`, `begin_show`,
  `_offer_start_anyway`) : quand un drone n'atteint pas son point de départ,
  l'IHM le nomme puis propose, au bout de 15 s, de lancer quand même. 
- **Le nombre maximal de drones simultanés** : Je n'ai pu faire voler que 2 drones simultanément contre 3 prévus par manque de disponibilité du 3e et par manque de temps.
- **Les dimensions utiles de la volière et l'autonomie typique** : [à compléter].

## Couplages non évidents

- **Les seuils de batterie viennent de l'airframe.** Ils sont lus dans la
  section `BAT` du fichier de configuration de chaque drone. Modifier un
  airframe change donc le comportement de l'IHM, sans qu'aucune ligne de code
  ne bouge. 
- **L'icône de bureau pointe vers un dépôt précis**, celui d'où
  `install_launcher.sh` a été lancé. Il n'existe qu'une entrée ; la relancer
  depuis un autre clone fait basculer l'icône, sans prévenir.
- **Les fichiers de `src/qt_gui/data/`** (scénarios composés, flotte mémorisée)
  sont exclus du suivi de version. Ils ne suivent pas un clone.

## Si je reprenais le projet demain

1. Le repli mocap → télémétrie, avec une alarme visible. C'est la seule limite
   qui peut tromper un opérateur en vol.
2. Valider le forçage de départ en simulation, puis en vol.
3. Chercher la cause du comportement du 110.
4. Reprendre la déconfliction sur plusieurs cycles de boucle, pas seulement le
   premier.

