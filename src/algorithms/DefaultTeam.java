package algorithms;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.TreeSet;

public class DefaultTeam {

    public static final double BUDGET = 1664;

    public Tree2D calculSteiner( ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints ) {

        // Application floyd-warshall sur tous les points et retourner la
        // matrice des dist et paths
        double[][] dist = calculDists( points, edgeThreshold );
        int[][] paths = calculPaths( points, edgeThreshold );

        // Construire le graphe pondéré complet K = (S, OS, w)
        List<Edge> K = this.getListEdge( hitPoints, points, dist );

        // Dans K, construire un arbre couvrant T0 de longeur totale des arêtes la plus petite possible.
        List<Edge> resKruskal = kruskal( K, hitPoints );

        // transformation de l'arbre couverant obtenu qui une liste d'arrete en un arbre Tree2D
        Tree2D T0 = edgeListToTree( resKruskal );

        // Parcours en largeur et échange des arêtes par les chemins dans paths
        List<Point> U = ParcoursEtEchange( T0, paths, points );

        // Construction de l'arbre H à partir des points U
        List<Edge> H = getListEdge( U );

        // Construction de l'arbre couvrant
        resKruskal = kruskal( H, U );

        // transformation de l'arbre couverant obtenu qui une liste d'arêtes en un arbre Tree2D T'
        Tree2D TPrime = edgeListToTree( resKruskal );

        return TPrime;
    }

    public Tree2D calculSteinerBudget( ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints ) {
        // Application floyd-warshall sur tous les points et retourner la matrice des dist et paths
        double[][] dist = calculDists( points, edgeThreshold );
        int[][] paths = calculPaths( points, edgeThreshold );

        // Construire le graphe pondéré complet K = (S, OS, w)
        List<Edge> K = this.getListEdge( hitPoints, points, dist );

        // Dans K, construire un arbre couvrant T0 de longeur totale des arêtes la plus petite possible.
        List<Edge> resKruskal = kruskal( K, hitPoints );
        
        // application de l'algorithme de prim
        List<Edge> calculPrim = prim( resKruskal, hitPoints.get( 0 ), BUDGET );

        // obtenir les arretes constituant le plus court chemin entre chacun des points
        List<Edge> realPath = setRealPath( calculPrim, paths, points );
        
        // transformation de l'arbre  obtenu qui est une liste d'arêtes en un arbre Tree2D
        Tree2D steinerBudget = edgeListToTree( realPath );

        return steinerBudget;
    }

    public int[][] calculPaths( List<Point> points, int edgeThreshold ) {

        int[][] paths = new int[points.size()][points.size()];
        for ( int i = 0; i < paths.length; i++ )
            for ( int j = 0; j < paths.length; j++ )
                paths[i][j] = i;

        double[][] dist = new double[points.size()][points.size()];

        for ( int i = 0; i < paths.length; i++ ) {
            for ( int j = 0; j < paths.length; j++ ) {
                if ( i == j ) {
                    dist[i][i] = 0;
                    continue;
                }
                if ( points.get( i ).distance( points.get( j ) ) <= edgeThreshold )
                    dist[i][j] = points.get( i ).distance( points.get( j ) );
                else
                    dist[i][j] = Double.POSITIVE_INFINITY;
                paths[i][j] = j;
            }
        }

        for ( int k = 0; k < paths.length; k++ ) {
            for ( int i = 0; i < paths.length; i++ ) {
                for ( int j = 0; j < paths.length; j++ ) {
                    if ( dist[i][j] > dist[i][k] + dist[k][j] ) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                        paths[i][j] = paths[i][k];

                    }
                }
            }
        }

        return paths;

    }

    public double[][] calculDists( ArrayList<Point> points, int edgeThreshold ) {
        double[][] dist = new double[points.size()][points.size()];

        for ( int i = 0; i < dist.length; i++ ) {
            for ( int j = 0; j < dist.length; j++ ) {
                double d = points.get( i ).distance( points.get( j ) );
                if ( d <= edgeThreshold ) {
                    dist[i][j] = d;
                } else {
                    dist[i][j] = Double.POSITIVE_INFINITY;
                }
            }
        }

        for ( int k = 0; k < dist.length; k++ ) {
            for ( int i = 0; i < dist.length; i++ ) {
                for ( int j = 0; j < dist.length; j++ ) {
                    if ( dist[i][j] > dist[i][k] + dist[k][j] ) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }

        return dist;
    }

    /**
     * Effectuer un parcours en largeur de l'arbre et remplacer toute arête u v
     * par un plus court chemin entre u et v dans G. Le resulat est l'ensemble
     * des points U.
     * 
     * @param t
     *            l'arbre à parcourir
     * @param paths
     *            matrice paths
     * @param points
     *            la liste de points de G
     * @return
     */
    private List<Point> ParcoursEtEchange( Tree2D t, int[][] paths, ArrayList<Point> points ) {
        // le résultat
        List<Point> res = new ArrayList<Point>();
        // on ajoute la racine de l'arbre
        res.add( t.getRoot() );
        // File
        Queue<Tree2D> file = new LinkedList<>();
        Tree2D temp = null;
        file.add( t );
        while ( file.size() != 0 ) {
            // Enfiler
            temp = file.poll();
            for ( Tree2D fils : temp.getSubTrees() ) {
                // Pour chaque fils : on ajoute les point intermedaire ( si il
                // en existe un autre chemin plus court entre les point
                // temp.getRoot() et fils.getRoot())
                res.addAll( shortestPath( points, paths, temp.getRoot(), fils.getRoot() ) );
                // on ajoute le point fils
                res.add( fils.getRoot() );
                // on met le fils dans la file pour etre parcouru après
                file.add( fils );
            }
        }
        return res;
    }

    /**
     * Retourne le plus court chemin pour aller de a vers b. Le résultat est
     * l'ensemble des points intermediaire entre a et b. si il y a un autre
     * chemin plus court ça renvoi une liste vide
     * 
     * @param points
     *            l'ensemble des points du graphe
     * @param paths
     *            la matrice paths
     * @param a
     *            point a
     * @param b
     *            point b
     * @return
     */
    private List<Point> shortestPath( List<Point> points, int[][] paths, Point a, Point b ) {
        // le resultat
        List<Point> p = new ArrayList<Point>();
        // on récupère l'index du point a
        int indexa = points.indexOf( a );
        // on récupère l'index du point b
        int indexb = points.indexOf( b );
        // on récupère le premier point intermediaire à visiter si on veut aller
        // du point a vers b
        int temp = paths[indexa][indexb];
        // on s'arrete lorsque on arrive au point b
        while ( temp != indexb ) {
            // on ajoute le point intermediaire au resultat
            p.add( points.get( temp ) );
            // on avance le point intermediaire
            temp = paths[temp][indexb];
        }

        return p;
    }

    /**
     * kruskal qui permet d'obtenir un arbre couverant, le resultat un arbe
     * Tree2D
     * 
     * @param grahpe
     *            le graphe sous forme d'une liste d'arretes
     * @param sommets
     *            les sommets du graphe
     * @return
     */
    private List<Edge> kruskal( List<Edge> grahpe, List<Point> sommets ) {
        // trier les arretes par ordre croissant
        Collections.sort( grahpe );
        // création de la map qui associe à un point une étiquète
        Map<Point, Integer> label = this.generateLabels( sommets );
        // le résultat
        List<Edge> solution = new ArrayList<Edge>();
        // on parcoure notre graphe
        for ( Edge e : grahpe ) {
            // avoir les meme étiquettes => ça va créer un cycle
            if ( label.get( e.getA() ) == label.get( e.getB() ) )
                continue;
            // on ajoute l'arrete au resultat
            solution.add( e );
            // récupérer l'étiquette du point a de l'arete e ajoutée
            Integer labelX = label.get( e.getA() );
            // récupérer étiquette du point b de l'arete e ajoutée
            Integer labelY = label.get( e.getB() );
            // re-étiquetter tous les points ayant étiquette x en y ( à partir
            // des aretes déjà ajoutés au résultat )
            for ( Edge e1 : solution ) {
                if ( label.get( e1.getA() ) == labelX ) {
                    label.replace( e1.getA(), labelX, labelY );
                }
                if ( label.get( e1.getB() ) == labelX ) {
                    label.replace( e1.getB(), labelX, labelY );
                }
            }
        }

        return solution;
    }

    /**
     * Transformation d'une liste d'arretes en un arbre Tree2D
     * 
     * @param list
     *            Le graphe sous forme d'une liste d'arretes
     * @return
     */
    private Tree2D edgeListToTree( List<Edge> list ) {
        // le resultat
        Tree2D res, temp;
        // file
        Queue<Tree2D> queue = new LinkedList<Tree2D>();
        // première arete de la edge list
        Edge e = list.get( 0 );
        // la racine de l'arbre
        res = new Tree2D( e.getA(), new ArrayList<Tree2D>() );
        // enfiler la racine dans la queue
        queue.add( res );
        
        while ( !queue.isEmpty() ) {
            // le premier élement de la file
            temp = queue.poll();
            // liste contenant les aretes consulté lors d'une itération => il faut les supprimer
            List<Edge> edgesToRemove = new ArrayList<>();
            // on parcourt notre graphe
            for ( int i = 0; i < list.size(); i++ ) {
                // iem arete
                Edge ed = list.get( i );
                // si l'une des extrémité de l'arete = au root de temp => l'autre extrémité est un fils 
                // on applique ceci pour l'extrémité a et b 
                if ( ed.getA().equals( temp.getRoot() ) ) {
                    Tree2D fils = new Tree2D( ed.getB(), new ArrayList<Tree2D>() );
                    // enfiler le fils dans la queue
                    queue.add( fils );
                    temp.getSubTrees().add( fils );
                    // arete déja consulté => ajout dans la liste
                    edgesToRemove.add( ed );
                }
                if ( ed.getB().equals( temp.getRoot() ) ) {
                    Tree2D fils = new Tree2D( ed.getA(), new ArrayList<Tree2D>() );
                    // enfiler le fils dans la queue
                    queue.add( fils );
                    temp.getSubTrees().add( fils );
                    // arete déja consulté => ajout dans la liste
                    edgesToRemove.add( ed );
                }
            }
            // supprimer les arretes déja consulté
            for ( Edge i : edgesToRemove ) {
                list.remove( i );
            }
        }
        return res;
    }

    /**
     * Obtenir la liste de toutes les arretes possible entre les points
     * 
     * @param points
     *            la liste des points
     * @return
     */
    private List<Edge> getListEdge( List<Point> points ) {
        // le resultat
        List<Edge> res = new ArrayList<Edge>();
        // on parcoure la liste des points avec deux boucles pour chaque point i
        // et j ( i != j ) on crée une arrete
        for ( int i = 0; i < points.size() - 1; i++ ) {
            for ( int j = i + 1; j < points.size(); j++ ) {
                res.add( new Edge( points.get( i ), points.get( j ) ) );
            }
        }
        return res;
    }

    /**
     * Obtenir un graphe complet dont les sommet se trouvent uniquement dans les
     * hitpoint (S) et que dans chaque arrete uv on stock la distance w défini
     * par la longeur du plus court chemin entre u et v dans la liste points
     * (G).
     * 
     * @param hitpoints
     *            les hitpoints (S)
     * @param points
     *            la liste des points (G)
     * @param dist
     *            matrice des distance
     * @return
     */
    private List<Edge> getListEdge( List<Point> hitpoints, List<Point> points, double[][] dist ) {
        // le resultat
        List<Edge> res = new ArrayList<Edge>();
        // on parcourt la liste des points de hitpoints pour chaque point i
        // et j ( i != j ) on crée une arete avec un constructeur qui prends
        // également un parametre distance
        // cette dernière est obtenu à partir de la matrice dist qui renvoi la
        // distance du plus court chemin entre i et j
        for ( int i = 0; i < hitpoints.size() - 1; i++ ) {
            for ( int j = i + 1; j < hitpoints.size(); j++ ) {
                res.add( new Edge( hitpoints.get( i ), hitpoints.get( j ),
                        dist[points.indexOf( hitpoints.get( i ) )][points.indexOf( hitpoints.get( j ) )] ) );
            }
        }
        return res;
    }

    /**
     * Attribuer pour chaque point une étiquète différente. l'étiquete est un
     * entier
     * 
     * @param points
     *            la liste des points
     * @return
     */
    private Map<Point, Integer> generateLabels( List<Point> points ) {
        // le résultat
        Map<Point, Integer> labelMap = new HashMap<>();
        // i represente l'étiquète on l'incrémente à chaque fois pour avoir une
        // nouvelle
        int i = 1;
        for ( Point p : points ) {
            labelMap.put( p, i++ );
        }
        return labelMap;
    }

    /**
     * Associer à chaque point une Heap contenant toute les arretes dans
     * lesquelles ce sommet fait parti La PriorityQueue permet d'avoir un Heap
     * trié par ordre croissant de distance d'une arrete, le premier élément est
     * toujours l'élement la distance la plus petite
     * 
     * @param list
     *            le graphe sous forme d'une liste d'arretes
     * @return
     */
    private Map<Point, PriorityQueue<Edge>> getPointEdgesMap( List<Edge> list ) {
        // le resultat
        Map<Point, PriorityQueue<Edge>> res = new HashMap<Point, PriorityQueue<Edge>>();
        // on parcourt la liste des arretes
        for ( Edge edge : list ) {

            // edge est une arete constitué du point a et b
            // première apparition du point a => création de la heap
            if ( !res.containsKey( edge.getA() ) ) {
                res.put( edge.getA(), new PriorityQueue<Edge>() );
            }
            // première apparition du point b => création de la heap
            if ( !res.containsKey( edge.getB() ) ) {
                res.put( edge.getB(), new PriorityQueue<Edge>() );
            }
            // ajoute de l'arete au heap associé au point a
            res.get( edge.getA() ).add( edge );
            // ajoute de l'arete au heap associé au point b
            res.get( edge.getB() ).add( edge );

        }

        return res;
    }

    /**
     * à partir des points déja sélectionné lors d'une itération de l'algorithme
     * de prim, on choisi l'arretes qui a cout minimal à ajouté
     * 
     * @param pointEdgesMap
     *            La map associant un point aux aretes dans lesquelles fait
     *            parti
     * @param addedPoints
     *            Les points déja ajouté lors de l'algorithme de prim
     * @return
     */
    private Edge getMinEdge( Map<Point, PriorityQueue<Edge>> pointEdgesMap, List<Point> addedPoints ) {
        // le resultat
        Edge res = null;
        // l'idée c'est qu'on a la map pointEdgesMap qui associe à un point x
        // une heap contenant toutes les aretes dont x est une extrimité.
        // cette heap nous garantie que la première arete est de distance
        // minimal, on parcourt tous les point
        // on récupère pour chaque point la première arete de sa heap, on les
        // stock dans la heap edgeCandidats.
        Queue<Edge> edgeCandidats = new PriorityQueue<Edge>();
        // on parcourt les points
        for ( Point p : addedPoints ) {
            // dans l'algo du prim on supprime les aretes retenu, du coup il se
            // peut
            // que la heap du point p est vide
            if ( pointEdgesMap.get( p ).isEmpty() )
                continue;
            // ajout de l'arete de distance min à edgeCandidats
            edgeCandidats.add( pointEdgesMap.get( p ).peek() );
        }
        // le premier élément de la heap est de distance minimal
        res = edgeCandidats.peek();
        return res;
    }

    /**
     * Implémentation de l'algo de prim en spécifiant un sommet de départ et un
     * budget à respecté. A chauqe itération on sélectionne l'arrete qui a le
     * cout le moins eleve parmis les points déja visités.
     * 
     * @param edgeList
     *            le graphe sous forme d'une liste d'arretes
     * @param sommetDepart
     *            maison mere
     * @param budget
     *            le budget qu'on doit pas dépasser
     * @return
     */
    private List<Edge> prim( List<Edge> edgeList, Point sommetDepart, double budget ) {
        // le resultat
        List<Edge> res = new ArrayList<Edge>();
        // obtenir la map associant un point p à une heap des aretes dont p est
        // une extrimité.
        Map<Point, PriorityQueue<Edge>> pointEdgesMap = getPointEdgesMap( edgeList );
        // les points ajoutés à l'arbre de steiner avec restriction budgétaire
        List<Point> addedPoints = new ArrayList<Point>();
        // on ajoute le premier point qui est maison-mère
        addedPoints.add( sommetDepart );
        // initialiser le cout
        double cost = 0.0;
        while ( true ) {
            // obtenir l'arete à ajoutée = temp
            Edge temp = getMinEdge( pointEdgesMap, addedPoints );
            // supprimer temp dans pointEdgesMap
            pointEdgesMap.get( temp.getA() ).remove( temp );
            pointEdgesMap.get( temp.getB() ).remove( temp );
            // ajouter les point a et b constituant l'arete temp
            if ( !addedPoints.contains( temp.getA() ) )
                addedPoints.add( temp.getA() );
            if ( !addedPoints.contains( temp.getB() ) )
                addedPoints.add( temp.getB() );

            // on test on dépasse le budget dans ce cas on sort
            if ( cost + temp.getDist() >= budget )
                break;
            // on incrémente le cout
            cost += temp.getDist();
            // on ajoute l'arete temp au resultat
            res.add( temp );
        }

        return res;
    }

    /**
     * Pour chaque arrete du graphe reliant les points a et b, remplacer cette
     * dernière par les arretes qui constitue le plus court chemin entre a et b
     * à partir de la matrice paths. Deux possiblité : soit cette arrete est
     * elle meme le plus court chemin ou soit il existe d'autre arretes reliant
     * les point a et b qui forment le plus court chemin dans ce cas on retourne
     * ces dernières.
     * 
     * @param edges
     *            le graphe sous forme d'une liste d'arretes
     * @param paths
     *            la matrice paths
     * @param points
     *            la liste des points
     * @return le nouveau graphe sous forme d'une liste d'arretes
     */
    private List<Edge> setRealPath( List<Edge> edges, int[][] paths, List<Point> points ) {
        // le resultat
        List<Edge> res = new ArrayList<Edge>();
        // on parcourt le graph
        for ( Edge e : edges ) {
            // pour chaque arete e du graphe : obtenir l'index de a et obtenir
            // l'index de b
            int a = points.indexOf( e.getA() );
            int b = points.indexOf( e.getB() );
            // on boucle tant que on est pas arriver au point b
            while ( a != b ) {
                // paths[a][b] nous renvoi l'index du prochaine sommet à visiter
                // (pour le plus court chemin)
                // il se peut qu'il soit directement b ou un autre sommet c
                // on  crée ensuite une arete reliant le point a et le prochaine
                // point, on l'ajoute au resultat
                res.add( new Edge( points.get( a ), points.get( paths[a][b] ) ) );
                // on avance le point a
                a = paths[a][b];
            }
        }

        return res;
    }

}