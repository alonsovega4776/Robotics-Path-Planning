
import Graph


G = Graph.Graph(directed=False)

v_0 = G.insert_vertex('a') #0
v_1 = G.insert_vertex('b') #1
v_2 = G.insert_vertex('c') #2
v_3 = G.insert_vertex('d') #3
v_4 = G.insert_vertex('e') #4


e_03 = G.insert_edge(G.get_vertex(0), G.get_vertex(3), 'ad')
e_30 = G.insert_edge(G.get_vertex(3), G.get_vertex(0), 'da')
e_01 = G.insert_edge(v_0, v_1, 'ab')
e_41 = G.insert_edge(G.get_vertex(4), G.get_vertex(1), 'eb')

for v in G.vertices():
    print('v_',v.id(),'  degree out: ', G.degree(v, direction='+'))
    print('v_',v.id(),'  degree in: ', G.degree(v, direction='-'))

print()

e = G.get_edge(v_0, v_1)
print(e.element())