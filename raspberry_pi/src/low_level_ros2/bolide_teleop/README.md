# Bolide Teleop

Ce package ROS2 permet le contrôle manuel de la voiture autonome Bolide via le clavier.

## Description

Le package fournit une interface de téléopération simple utilisant les touches du clavier pour contrôler la vitesse et la direction de la voiture. Il est conçu pour les tests et la conduite manuelle pendant le développement.

## Fonctionnement

Le nœud `teleop_keyboard` capture les entrées clavier et convertit les commandes en valeurs normalisées pour la vitesse et la direction :

- **Flèche haut** : Augmente la vitesse en avant
- **Flèche bas** : Augmente la vitesse en arrière
- **Flèche gauche** : Tourne à gauche
- **Flèche droite** : Tourne à droite
- **S** : Frein d'urgence (arrêt immédiat)  TODO: implémenter
- **N** : Position neutre (vitesse et direction à zéro)
- **Q** : Quitter le programme

Les valeurs sont incrémentées par pas fixes et limitées aux plages autorisées.

## Paramètres

- `debug` : Active les logs de debug (défaut : False)

## Topics

### Publishers
- `/cmd_vel` (std_msgs/Float32) : Vitesse de la voiture (-1.0 à 1.0)
- `/cmd_dir` (std_msgs/Float32) : Direction de la voiture (-1.0 à 1.0)

## Lancement

```bash
ros2 run bolide_teleop teleop_keyboard
```

Ou via le launch file complet :
```bash
ros2 launch bolide_teleop teleop_keyboard.launch
```

Le launch file inclut également le démarrage des nœuds nécessaires (Lidar, STM32, direction).

## Architecture

Ce package fait partie du niveau bas (low_level) et publie directement les commandes de mouvement. Il est utilisé pour :

- Les tests manuels de la voiture (calibrer la vitesse et la direction notamment)
- Le développement et débogage


## Remarques importantes

- Pour passer de marche avant à marche arrière, il est nécessaire d'utiliser d'abord la touche 'N' pour la position neutre, plusieurs appuis sont en général nécessaires.
- Les valeurs maximales (1.0 et -1.0) causent des problèmes de tension sur le Raspberry Pi