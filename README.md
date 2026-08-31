# quad4d_rebooted - Click'n Fly

Click'nFly is a trajectory editor/generator and a flight director for quadrotor drones.

It is intended as a tool for drone demonstrations in ENAC's indoor flight arena but might fit other applications.
It is for now writen in Python, with the Graphical User Interface leveraging PyQT.


Application de conduite de shows de drones en volière. Elle génère des
trajectoires chorégraphiées pour plusieurs quadrirotors, les déconflicte avant
le vol, et les fait voler depuis une interface unique : préparer, lancer,
suivre et arrêter le show sans passer par la ligne de commande.

Elle s'appuie sur l'autopilote [Paparazzi](https://github.com/paparazzi/paparazzi)
pour le vol et sur la bibliothèque [pat](https://github.com/poine/pat) pour les
trajectoires. La position des drones vient du système de capture de mouvement
de la volière, pas du GPS.

## Prérequis

Trois choses, à installer séparément — ce dépôt ne contient que l'application :

| | où le prendre |
|---|---|
| Python 3.10 ou plus | le gestionnaire de paquets du système |
| `pat`, le module `pat3` | <https://github.com/poine/pat> |
| Paparazzi, avec `sw/lib/python` | <https://github.com/paparazzi/paparazzi> |

Il faut également une volière équipée OptiTrack, diffusant des messages
`EXTERNAL_POSE`.

## Installation

**1. Python.** Vérifiez d'abord ce que vous avez :

```bash
python3 --version
```

S'il manque, ou s'il est antérieur à 3.10, sur Debian ou Ubuntu :

```bash
sudo apt install python3 python3-venv python3-pip
```

**2. La bibliothèque pat.** Elle n'est pas sur PyPI : on la clone, et on
déclarera son chemin à l'étape 4. L'emplacement est libre, `~/work/pat` est
la convention du laboratoire :

```bash
mkdir -p ~/work && git clone https://github.com/poine/pat.git ~/work/pat
```

Attention au nom : le dépôt s'appelle `pat`, le module Python `pat3`. Et ce
n'est pas le même dépôt que celui de cette application, bien que tous deux
soient de Antoine Drouin.

**3. L'environnement Python.** Le lanceur cherche `~/venv_quad4d` par défaut :

```bash
python3 -m venv ~/venv_quad4d
source ~/venv_quad4d/bin/activate
pip install pyyaml numpy scipy matplotlib pyside6 numpy_stl pyqtgraph pyopengl ivy-python lxml
```

**4. Les chemins vers pat et Paparazzi.** Un lancement par icône ne lit pas
votre `~/.bashrc` : le `PYTHONPATH` doit donc être déclaré dans un fichier
dédié, `~/.config/clicknfly.env`, que le lanceur charge à chaque démarrage.

```bash
mkdir -p ~/.config
cat > ~/.config/clicknfly.env <<'EOF'
export PYTHONPATH="$PYTHONPATH:$HOME/work/pat"
export PYTHONPATH="$PYTHONPATH:/chemin/vers/paparazzi/sw/lib/python"
export PAPARAZZI_HOME="/chemin/vers/paparazzi"
EOF
```

La première ligne suppose le clone de l'étape 2 ; adaptez-la si vous l'avez mis
ailleurs, et remplacez le chemin de Paparazzi par le vôtre. C'est l'erreur la
plus fréquente au premier lancement : sans ces chemins, l'application s'arrête
sur un `ModuleNotFoundError: pat3`.

Pour vérifier, sans quitter le venv :

```bash
source ~/.config/clicknfly.env && python3 -c "import pat3; print(pat3.__file__)"
```

**5. L'icône de bureau.** Une seule commande, à lancer depuis la racine du
dépôt :

```bash
./install_launcher.sh
```

Elle écrit `~/.local/share/applications/clicknfly.desktop` avec des chemins
absolus résolus depuis l'emplacement du dépôt. « Click'n Fly » apparaît alors
dans le menu des applications, et peut être épinglé.

Le script résout tout seul le chemin du dépôt : si vous le lancez depuis un
autre clone, l'icône bascule vers celui-là. Il n'existe qu'une entrée de bureau,
la précédente est remplacée.

## Lancer

Par l'icône, ou en console :

```bash
source ~/venv_quad4d/bin/activate
cd src/qt_gui && ./click_n_fly.py
```

Options utiles :

| option | effet |
|---|---|
| `-v`, `--verbose` | détail de développement : mode de transit retenu, étagement, ordonnancement |
| `--scen NOM` | démarrer directement sur un scénario |

Sans `-v`, le journal reste à l'essentiel : avertissements et étapes clés.

**Un lancement par icône n'a pas de terminal où écrire.** En cas d'échec, une
fenêtre d'erreur apparaît, et le journal complet est dans :

```bash
tail -30 ~/.cache/clicknfly.log
```

## Avant un vol en volière

Trois points conditionnent une démonstration, et aucun n'est détecté par
l'application :

- **La configuration de télémétrie doit être allégée.** Avec la configuration
  par défaut, le volume de messages émis par les drones sature la liaison au
  détriment des positions issues de la capture de mouvement, et les commandes
  ne passent plus correctement.
- **Chaque drone doit être appairé à sa propre radiocommande.** C'est une
  exigence de sécurité : sans elle, le drone ne vole pas.
- **Chaque drone doit embarquer le bon firmware** pour accepter le mode guidé,
  faute de quoi il reste en mode NAV. Le cas échéant, reprogrammer l'autopilote.

Les seuils de batterie ne sont pas dans le code : ils sont lus dans la section
`BAT` du fichier `airframe` de chaque drone, celui-là même qu'utilise
l'autopilote. Changer un seuil ne demande donc aucune modification du logiciel.

## Où trouver quoi

| chemin | contenu |
|---|---|
| `src/qt_gui/click_n_fly.py` | l'application |
| `src/qt_gui/traj_factory.py` | les figures |
| `src/qt_gui/scenarios.py` | les scénarios prédéfinis |
| `src/qt_gui/spatial_deconfliction.py` | la déconfliction par ordonnancement |
| `src/qt_gui/data/` | scénarios composés par l'opérateur, propres à la machine |
| `docs/concept_operationnel.md` | le concept d'opérations |
| `docs/trajectories.md` | les trajectoires |
| `docs/TODO.md` | les chantiers ouverts |

Les fichiers de `src/qt_gui/data/` sont exclus du suivi de version : ils sont
locaux à chaque installation. Un nouveau clone démarre donc sans les scénarios
personnalisés du précédent.


trajectoires et guidage 3D+t pour quadrirotors

doc is [here](https://poine.github.io/quad4d_rebooted).
