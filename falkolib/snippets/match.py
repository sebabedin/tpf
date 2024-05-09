'''
Created on May 2, 2024

@author: seb
'''

class Node(object):
    
    def __init__(self, idx):
        self.idx = idx
        self.matchs = []

if __name__ == '__main__':
    nodes = []
    for loop in range(5):
        print(f'iter {loop}')
        nodes.append(Node(loop))
    
        if 1 < len(nodes):
            node1 = nodes[-1]
            for i in range(0, len(nodes)-1):
                node2 = nodes[i]
                print(f'{node1.idx} {node2.idx}')
                # for j in range(i+1, len(nodes)):
                #     node2 = nodes[j]
                #     print(f'{node1.idx} {node2.idx}')        
        
    
    