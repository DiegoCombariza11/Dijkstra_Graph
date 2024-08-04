import sys

class Grafo:
    def __init__(self, vertices):
        self.V = vertices
        self.grafo = [[0 for columna in range(vertices)] for fila in range(vertices)]

    def imprimir_matriz_adyacencia(self):
        print("Matriz de adyacencia:")
        for fila in self.grafo:
            print(" ".join(map(str, fila)))

    def imprimir_lista_nodos(self):
        print("Lista de nodos con sus índices:")
        for i in range(self.V):
            print(f"Nodo {i}: {i}")

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
                    if dist[v] > dist[u] + self.grafo[u][v]:
                        dist[v] = dist[u] + self.grafo[u][v]
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
        camino = []
        while j != -1:
            camino.append(j)
            j = padres[j]
        camino.reverse()
        print(" -> ".join(map(str, camino)))

def main():
    try:
        vertices = int(input("Ingrese el tamaño 'n'  de la matriz de adyacencia (nxn): "))
        if vertices <= 0:
            raise ValueError("El tamaño de la matriz debe ser mayor que 0.")
    except ValueError as e:
        print(e)
        return

    g = Grafo(vertices)

    print("Ingrese el grafo como una matriz de adyacencia (fila por fila) separado nodo por nodo por un espacio:")
    for i in range(vertices):
        while True:
            try:
                fila = list(map(int, input(f"Fila {i + 1}: ").split()))
                if len(fila) != vertices:
                    raise ValueError(f"Debe ingresar exactamente {vertices} valores.")
                g.grafo[i] = fila
                break
            except ValueError as e:
                print(e)


    g.imprimir_matriz_adyacencia()
    g.imprimir_lista_nodos()

    try:
        src = int(input(f"Ingrese el índice del vértice de origen (de 0 a {vertices - 1}): "))
        dest = int(input(f"Ingrese el índice del vértice de destino (de 0 a {vertices - 1}): "))
        if src < 0 or src >= vertices or dest < 0 or dest >= vertices:
            raise ValueError("El vértice de origen y destino deben estar dentro del rango válido.")
    except ValueError as e:
        print(e)
        return

    dist, padres = g.dijkstra(src)
    g.imprimir_solucion(dist, padres, src, dest)

if __name__ == "__main__":
    main()