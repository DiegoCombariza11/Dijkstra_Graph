import sys

class Grafo:
    def __init__(self, vertices):
        self.V = vertices
        self.grafo = [[0 for columna in range(vertices)] for fila in range(vertices)]
    
    def imprimir_matriz_adyacencia(self):
        for fila in self.grafo:
            print(" ".join(map(str, fila)))

    def distancia_minima(self, dist, spt_set):
        minimo = sys.maxsize
        indice_minimo = -1

        for v in range(self.V):
            if dist[v] < minimo and not spt_set[v]:
                minimo = dist[v]
                indice_minimo = v

        return indice_minimo

    def dijkstra(self, src):
        dist = [sys.maxsize] * self.V
        dist[src] = 0
        spt_set = [False] * self.V
        padres = [-1] * self.V

        for cout in range(self.V):
            u = self.distancia_minima(dist, spt_set)
            if u == -1:  # No hay más vértices alcanzables
                break
            spt_set[u] = True

            for v in range(self.V):
                if self.grafo[u][v] > 0 and not spt_set[v]:
                    if dist[v] > dist[u] + 1:  # Distancia es siempre 1 si hay una conexión
                        dist[v] = dist[u] + 1
                        padres[v] = u

        return dist, padres

    def imprimir_solucion(self, dist, padres, src, dest):
        if dist[dest] == sys.maxsize:
            print(f"No hay un camino desde el vértice {src} hasta el vértice {dest}")
        else:
            print("Distancia desde el origen hasta el destino:", dist[dest])
            print("Camino:")
            self.imprimir_camino(padres, dest)
            print()

    def imprimir_camino(self, padres, j):
        if padres[j] == -1:
            print(j, end=" ")
            return
        self.imprimir_camino(padres, padres[j])
        print(j, end=" ")

def main():
    vertices = int(input("Ingrese el número de vértices: "))
    g = Grafo(vertices)

    print("Ingrese el grafo como una matriz de adyacencia (fila por fila):")
    for i in range(vertices):
        fila = list(map(int, input().split()))
        g.grafo[i] = fila

    print("Matriz de adyacencia:")
    g.imprimir_matriz_adyacencia()

    src = int(input("Ingrese el vértice de origen (de 0 a {}): ".format(vertices - 1)))
    dest = int(input("Ingrese el vértice de destino (de 0 a {}): ".format(vertices - 1)))
    
    dist, padres = g.dijkstra(src)
    g.imprimir_solucion(dist, padres, src, dest)

if __name__ == "__main__":
    main()
