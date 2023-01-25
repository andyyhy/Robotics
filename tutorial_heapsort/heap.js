/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {};

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    var element_index = heap.length;
    var parent_index = Math.floor((element_index - 1) / 2);
    heap.push(new_element);
    var done = (element_index <= 0) || (heap[parent_index] <= heap[element_index]);

    while (!done) {
        var temp = heap[parent_index];
        heap[parent_index] = heap[element_index];
        heap[element_index] = temp;

        element_index = parent_index;
        parent_index = Math.floor((element_index - 1) / 2);

        done = (element_index <= 0) || (heap[parent_index] <= heap[element_index]);
    }
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
    var ans = heap[0];
    heap[0] = heap[heap.length - 1];
    heap.pop()
    var k = 1;
    while (2 * k <= heap.length) {
        var j = 2 * k;
        if (j < heap.length && heap[j - 1] > heap[j]) {
            j++;
        }
        if (heap[k - 1] <= heap[j - 1]) {
            break;
        }
        var temp = heap[k - 1];
        heap[k - 1] = heap[j - 1];
        heap[j - 1] = temp;
        k = j;
    }
    return ans;

}

// assign extract function within minheaper object

// STENCIL: ensure extract method is within minheaper object
minheaper.extract = minheap_extract;






