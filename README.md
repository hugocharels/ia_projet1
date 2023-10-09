# INFO-F-311: Intelligence Artificielle
# Projet 1: Recherche
### Hugo Charels - 000544051
### 08 Octobre 2023

## Introduction

Dans ce projet, plusieurs heuristiques ont été développées. Une ou plus pour chaque problème. Il y a 3 catégories d'heuristiques, celles non-admissible, celles admissibles et celles consistante.
Une heuristique $h$ est admissible quand pour chaque état $n$, $h(n)$ ne dépasse jamais $h^{*}$ le coût optimal. ($0 \leq h(n) \leq h^{*}$). Si cette inégalité n’est pas respectée pour chaque état alors l’heuristique est dite inadmissible. 
Une heuristique consistante est une heuristique admissible qui pour chaque état $n$, $h(n) - h(n+1) \leq c(n,n+1)$ c’est à dire que l’heuristique de l’état $n$ moins celle de l’état suivant $n+1$ doit être inférieure ou égale au coût pour passer de l’état $n$ à l’état $n+1$.

## Heuristiques développées

- #### SimpleSearchProblem :
	Pour ce problème, l’heuristique développée est simplement la distance de Manhattan entre la position de l’agent et la position de la case de sortie.
	Si le jeu contient plusieurs agents et donc plusieurs sorties, pour chaque agent, on calcule la distance de Manhattan entre l’agent et la sortie la plus proche, puis on prend la valeur maximale de ces valeurs calculées pour chaque agent. Prendre le minimum à la place du maximum aurait aussi fonctionné, mais moins bien. Ces heuristiques sont consistantes.
	Une autre alternative envisageable était de prendre la somme de ces valeurs, mais alors l’heuristique n’est pas admissible, car $\sum^{k}_{i=1} h_{i}$ peut-être plus grand que $h^{*}$, étant le vrai coût optimal.

- #### CornerSearchProblem :
	L'heuristique développée est consistante. Elle est divisée en 3 partie, la première va calculer la distance de Manhattan la plus petite entre chaque agent et les sorties et va prendre le maximum de ces valeurs. Puis en deuxième, on prend la distance de Manhattan entre les corners restant divisé par le nombre d'agents. En dernier lieu, on va calculer la distance la plus petite entre les corners restants et les cases de sorties.
	L'addition de ces 3 valeurs donne cette heuristique.
	Étant donné que cette heuristique est consistante et que la recherche s'exécute très rapidement un autre heuristique admissible ou pas n'est pas utile, mais avant de trouver cette manière de faire simplement je calculais la distance de Manhattan vers le corner le plus proche et une fois tout les corners passé la distance de Manhattan par rapport aux cases de sortie. Cette heuristique est admissible, mais pas consistante, la recherche s'exécute aussi très rapidement, mais le nombre de nœuds étendu est beaucoup plus grand.

 - #### GemSearchProblem :
	L'heuristique développée est également consistante et est basée sur celle du "CornerSearchProblem". La première étape est identiquement la même, mais pour les positions des gems et non des corners. La deuxième étape, on va juste prendre le nombre de gems $- 1$ divisé par le nombre d'agents. Et en dernier, on ajoute le minimum entre les distances maximales des gems et des sorties. Une autre heuristique consistante est le nombre de gems restantes divisé par le nombre d'agents, mais le nombre de nœuds étendu est énormément plus grand. Une heuristique non-admissible et qui fonctionne très bien est bêtement le nombre de gems multiplié par 10. Mais le nombre d'actions de la solution est beaucoup plus grand.

## Comparaison des algorithmes

- #### Simple Search
| Niveau 3 | Nombre d’actions | Nombre de nœuds étendu |
| -------- | ---------------- | ---------------------- |
| DFS      | $276$            | $376$                  |
| BFS      | $10$             | $9610$                 |
| A*       | $10$             | $10$                   |

- #### Corner Search
| Niveau 3 | Nombre d’actions | Nombre de nœuds étendu |
| -------- | ----------------- | ----------------- |
| DFS      | $272$             | $375$             |
| BFS      | $28$              | $430558$          |
| A*       | $28$              | $357$             |

- #### Gems Search
| Niveau 3 | Nombre d’actions | Nombre de nœuds étendu |
| -------- | ----------------- | ----------------- |
| DFS      | $276$             | $376$             |
| BFS      | $14$              | $26061$           |
| A*       | $14$              | $100$             |

Le DFS est efficace pour trouver un chemin rapidement, mais la longueur de ce chemin dépend du problème concerné, on peut donc considérer qu'elle est aléatoire. Le BFS lui trouve quoi qu'il arrive le meilleur chemin, car il parcourt le graphe des possibilités en largeurs. Le A* lui donne le meilleur chemin dépendamment de l'éfficacité de l'heuristique et idem pour le nombre de nœuds étendu. Dans ces cas, si l'heuristique est consistante donc le nombre d'action est minimal.
