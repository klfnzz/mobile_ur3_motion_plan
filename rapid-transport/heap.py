import numpy as np

class Heap():
    """Heap is a class of binary min. heap.
       Attributes:
           data    -- a list of data
           size    -- the number of data in the list
           n_levels -- level of the heap
           index -- a list of index of each member in the heap
    """
    
    #_#_#https://www.cnblogs.com/kkun/archive/2011/11/23/2260286.html

    def __init__(self, data_list):
        self.data = data_list
        self.size = len(data_list)
        self.n_levels = int(np.floor(np.log2(self.size))) + 1
        #_#_# The floor of the scalar x is the largest integer i, such that i <= x.
        self.index = list(range(self.size))
        self.build_heap()


    def build_heap(self):
        """
        build_heap heapifies from the second lowest level of the heap
        """
        for i in reversed(list(range(self.n_levels - 1))):
            for j in range(int(2**i)):
                self.heapify(int(2**i - 1 + j))


    def extract_min(self):
        if(self.size > 0):
            self.swap(0, self.size - 1)
            #_#_#self.index[i] = self.index[j]
            min_element = self.data[self.index[self.size - 1]]
            index = self.index.pop()
            #_#_#Remove the item at the given position in the list, and return it
            #_#_#If no index is specified
            #_#_#a.pop() removes and returns the last item in the list.
            self.size -= 1
            if(self.size == 0):
                self.n_levels = 0
            else:
                self.n_levels = int(np.floor(np.log2(self.size))) + 1
                self.heapify(0)
            return [index, min_element]
        else:
            print ("The heap is empty.")
            

    def decrease_key():
        pass


    def heapify(self, i):
        lc = self.left_child(i)
        rc = self.right_child(i)
        smallest = i
        if lc < self.size:
            if self.data[self.index[lc]] < self.data[self.index[smallest]]:
                smallest = lc
        if rc < self.size:
            if self.data[self.index[rc]] < self.data[self.index[smallest]]:
                smallest = rc
        if smallest != i:
            self.swap(i, smallest)            
            self.heapify(smallest)
    

    def parent(self, i):
        return int(i - 1)/2


    def left_child(self, i):
        return int(2*i) + 1
    

    def right_child(self, i):
        return int(2*i) + 2
        

    def swap(self, i, j):
        temp = self.index[i]
        self.index[i] = self.index[j]
        self.index[j] = temp


    def print_heap(self):
        size = self.size
        for i in range(self.n_levels):
            space = 10*int(2**(self.n_levels - i - 1)) - 1
            string = " "*space
            for j in range(int(2**i)):
                if(size > 0):
                    string += ''.join(str(self.data[self.index[int(2**i - 1 + j)]]) + " "*(2*space))
                    size -= 1
                else:
                    break
            print (string)
            print ("\n")
