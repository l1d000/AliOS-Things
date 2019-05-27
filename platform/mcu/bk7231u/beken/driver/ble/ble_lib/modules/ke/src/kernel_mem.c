/**
 ****************************************************************************************
 *
 * @file kernel_mem.c
 *
 * @brief Implementation of the heap management module.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MEM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>             // standard definition
#include <stdbool.h>            // standard boolean
#include "architect.h"               // architecture

#include "common_math.h"            // computation utilities
#include "RomCallFlash.h"
#include "kernel_config.h"          // kernel configuration
#include "kernel_mem.h"             // kernel memory
#include "kernel_env.h"             // kernel environment

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */
/// Pattern used to check if memory block is not corrupted
#define KERNEL_LIST_PATTERN           (0xA55A)
#define KERNEL_ALLOCATED_PATTERN      (0x8338)
#define KERNEL_FREE_PATTERN           (0xF00F)

/*
 * LOCAL TYPE DEFINITIONS
 ****************************************************************************************
 */
/// Free memory block delimiter structure (size must be word multiple)
/// Heap can be represented as a doubled linked list.
struct mblock_free
{
    /// Used to check if memory block has been corrupted or not
    uint16_t corrupt_check;
    /// Size of the current free block (including delimiter)
    uint16_t free_size;
    /// Next free block pointer
    struct mblock_free* next;
    /// Previous free block pointer
    struct mblock_free* previous;
};

/// Used memory block delimiter structure (size must be word multiple)
struct mblock_used
{
    /// Used to check if memory block has been corrupted or not
    uint16_t corrupt_check;
    /// Size of the current used block (including delimiter)
    uint16_t size;
};


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 * Check if memory pointer is within heap address range
 *
 * @param[in] type Memory type.
 * @param[in] mem_ptr Memory pointer
 * @return True if it's in memory heap, False else.
 */
static bool kernel_mem_is_in_heap(uint8_t type, void* mem_ptr)
{
    bool ret = false;
    uint8_t* block = (uint8_t*)kernel_env.heap[type];
    uint32_t size = kernel_env.heap_size[type];

    if((((uint32_t)mem_ptr) >= ((uint32_t)block))
            && (((uint32_t)mem_ptr) <= (((uint32_t)block) + size)))
    {
        ret = true;
    }

    return ret;
}


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
 void kernel_mem_init(uint8_t type, uint8_t* heap, uint16_t heap_size)
{
    // align first free descriptor to word boundary
    #if CPU_WORD_SIZE == 4
    kernel_env.heap[type] =  (struct mblock_free*)COMMON_ALIGN4_HI((uint32_t)heap);
    #elif CPU_WORD_SIZE == 2
    kernel_env.heap[type] =  (struct mblock_free*)COMMON_ALIGN2_HI((uint16_t)heap);
    #else
    #error No word size defined
    #endif // CPU_WORD_SIZE

    // protect accesses to descriptors
    GLOBAL_INT_DIS();

    // initialize the first block
    //  + compute the size from the last aligned word before heap_end
    kernel_env.heap[type]->free_size = ((uint32_t)&heap[heap_size] & (~3)) - (uint32_t)(kernel_env.heap[type]);
    kernel_env.heap[type]->corrupt_check = KERNEL_LIST_PATTERN;
    kernel_env.heap[type]->next = NULL;
    kernel_env.heap[type]->previous = NULL;

    kernel_env.heap_size[type] = heap_size;

//    dbg_mem.cpt = 0;

    // end of protection
    GLOBAL_INT_RES();
}


bool kernel_mem_is_empty(uint8_t type)
{
    bool result = false;
    struct mblock_free *first;
    uint8_t* block = NULL;
    /// Size of the current free block (including delimiter)
    uint32_t size;

    // select which memory block to check.
    block = (uint8_t*)kernel_env.heap[type];
    size = kernel_env.heap_size[type];

    // align first free descriptor to word boundary
    first =  kernel_env.heap[type];

    // protect accesses to descriptors
    GLOBAL_INT_DIS();

    // calculate size if whole memory block is free.
    size = ((uint32_t)&block[size] & (~3)) - (uint32_t)first;

    // compare it with available memory size of first block.
    if(first->free_size == size)
    {
        result = true;
    }

    // end of protection
    GLOBAL_INT_RES();

    // return result.
    return result;
}


bool kernel_check_malloc(uint32_t size, uint8_t type)
{
    struct mblock_free *node = NULL,*found = NULL;
    uint8_t cursor = 0;
    uint32_t totalsize;

    // compute overall block size (including requested size PLUS descriptor size)
    totalsize = COMMON_ALIGN4_HI(size) + sizeof(struct mblock_used);
    if(totalsize < sizeof(struct mblock_free))
    {
        totalsize = sizeof(struct mblock_free);
    }

    // sanity check: the totalsize should be large enough to hold free block descriptor
    ASSERT_ERR(totalsize >= sizeof(struct mblock_free));

    // protect accesses to descriptors
    GLOBAL_INT_DIS();

    while((cursor < KERNEL_MEM_BLOCK_MAX)&& (found == NULL))
    {
        uint8_t heap_id = ((cursor + type) % KERNEL_MEM_BLOCK_MAX);

        // Select Heap to use, first try to use current heap.
        node = kernel_env.heap[heap_id];
        ASSERT_ERR(node != NULL);

        // go through free memory blocks list
        while (node != NULL)
        {
            ASSERT_ERR(node->corrupt_check == KERNEL_LIST_PATTERN);

            // check if there is enough room in this free block
            if (node->free_size >= (totalsize))
            {
                if ((node->free_size >= (totalsize + sizeof(struct mblock_free)))
                        || (node->previous != NULL))
                {
                    // if a match was already found, check if this one is smaller
                    if ((found == NULL) || (found->free_size > node->free_size))
                    {
                        found = node;
                    }
                }
            }

            // move to next block
            node = node->next;
        }

        // Update size to use complete list if possible.
        if(found != NULL)
        {
            if (found->free_size < (totalsize + sizeof(struct mblock_free)))
            {
                totalsize = found->free_size;
            }
        }


        // increment cursor
        cursor ++;
    }

    // end of protection (as early as possible)
    GLOBAL_INT_RES();

    return (found != NULL);
}


void *kernel_malloc(uint32_t size, uint8_t type)
{
    struct mblock_free *node = NULL,*found = NULL;
    uint8_t cursor = 0;
    struct mblock_used *alloc = NULL;
    uint32_t totalsize;


    // compute overall block size (including requested size PLUS descriptor size)
    totalsize = COMMON_ALIGN4_HI(size) + sizeof(struct mblock_used);
    if(totalsize < sizeof(struct mblock_free))
    {
        totalsize = sizeof(struct mblock_free);
    }

    // sanity check: the totalsize should be large enough to hold free block descriptor
    ASSERT_ERR(totalsize >= sizeof(struct mblock_free));

    // protect accesses to descriptors
    GLOBAL_INT_DIS();

    while((cursor < KERNEL_MEM_BLOCK_MAX)&& (found == NULL))
    {
        uint8_t heap_id = ((cursor + type) % KERNEL_MEM_BLOCK_MAX);
        #if KERNEL_PROFILING
        uint32_t totalfreesize = 0;
        #endif //KERNEL_PROFILING

        // Select Heap to use, first try to use current heap.
        node = kernel_env.heap[heap_id];
        ASSERT_ERR(node != NULL);

        // go through free memory blocks list
        while (node != NULL)
        {
            ASSERT_ERR(node->corrupt_check == KERNEL_LIST_PATTERN);
            #if KERNEL_PROFILING
            totalfreesize += node->free_size;
            #endif //KERNEL_PROFILING

            // check if there is enough room in this free block
            if (node->free_size >= (totalsize))
            {
                if ((node->free_size >= (totalsize + sizeof(struct mblock_free)))
                        || (node->previous != NULL))
                {
                    // if a match was already found, check if this one is smaller
                    if ((found == NULL) || (found->free_size > node->free_size))
                    {
                        found = node;
                    }
                }
            }

            // move to next block
            node = node->next;
        }

        // Update size to use complete list if possible.
        if(found != NULL)
        {
            if (found->free_size < (totalsize + sizeof(struct mblock_free)))
            {
                totalsize = found->free_size;
            }
        }

        #if KERNEL_PROFILING
        kernel_env.heap_used[heap_id] = kernel_env.heap_size[heap_id] - totalfreesize;
        if(found != NULL)
        {
            kernel_env.heap_used[heap_id] += totalsize;
        }
        #endif //KERNEL_PROFILING

        // increment cursor
        cursor ++;
    }

    #if KERNEL_PROFILING
    {
        // calculate max used size
        uint32_t totalusedsize = 0;
        for(cursor = 0 ; cursor < KERNEL_MEM_BLOCK_MAX ; cursor ++)
        {
            totalusedsize +=  kernel_env.heap_used[cursor];
        }

        if(kernel_env.max_heap_used < totalusedsize)
        {
            kernel_env.max_heap_used = totalusedsize;
        }
    }
    #endif //KERNEL_PROFILING

    // Re-boot platform if no more empty space
    if(found == NULL)
    {
        bk_printf("kernel malloc failed!\r\n");
        bk_printf("type : %x\r\n", type);
        platform_reset(RESET_MEM_ALLOC_FAIL);
				//rom_env.platform_reset(RESET_MEM_ALLOC_FAIL);
    }
    else
    {
        // sublist completely reused
        if (found->free_size == totalsize)
        {
            ASSERT_ERR(found->previous != NULL);

            // update double linked list
            found->previous->next = found->next;
            if(found->next != NULL)
            {
                found->next->previous = found->previous;
            }

            // compute the pointer to the beginning of the free space
            #if CPU_WORD_SIZE == 4
            alloc = (struct mblock_used*) ((uint32_t)found);
            #elif CPU_WORD_SIZE == 2
            alloc = (struct mblock_used*) ((uint16_t)found);
            #endif
        }
        else
        {
            // found a free block that matches, subtract the allocation size from the
            // free block size. If equal, the free block will be kept with 0 size... but
            // moving it out of the linked list is too much work.
            found->free_size -= totalsize;

            // compute the pointer to the beginning of the free space
            #if CPU_WORD_SIZE == 4
            alloc = (struct mblock_used*) ((uint32_t)found + found->free_size);
            #elif CPU_WORD_SIZE == 2
            alloc = (struct mblock_used*) ((uint16_t)found + found->free_size);
            #endif
        }

        // save the size of the allocated block
        alloc->size = totalsize;
        alloc->corrupt_check = KERNEL_ALLOCATED_PATTERN;

        // move to the user memory space
        alloc++;
    }

    // end of protection (as early as possible)
    GLOBAL_INT_RES();
    ASSERT_ERR(node == NULL);

//    debug_mem_set((uint32_t*)alloc,type);

    return (void*)alloc;
}

void kernel_free(void* mem_ptr)
{
    struct mblock_free *freed;
    struct mblock_used *bfreed;
    struct mblock_free *node, *next_node, *prev_node;
    uint32_t size;
    uint8_t cursor = 0;

    // sanity checks
    ASSERT_INFO(mem_ptr != NULL, mem_ptr, 0);
    
//    debug_mem_reset((uint32_t*)mem_ptr);
    // point to the block descriptor (before user memory so decrement)
    bfreed = ((struct mblock_used *)mem_ptr) - 1;

    // check if memory block has been corrupted or not
    ASSERT_INFO(bfreed->corrupt_check == KERNEL_ALLOCATED_PATTERN, bfreed->corrupt_check, mem_ptr);
    // change corruption token in order to know if buffer has been already freed.
    bfreed->corrupt_check = KERNEL_FREE_PATTERN;

    // point to the first node of the free elements linked list
    size = bfreed->size;
    node = NULL;

    freed = ((struct mblock_free *)bfreed);

    // protect accesses to descriptors
    GLOBAL_INT_DIS();

    // Retrieve where memory block comes from
    while(((cursor < KERNEL_MEM_BLOCK_MAX)) && (node == NULL)){

        if(kernel_mem_is_in_heap(cursor, mem_ptr))
        {
            // Select Heap to use, first try to use current heap.
            node = kernel_env.heap[cursor];
        }
        else
        {
            cursor ++;
        }
    }

    // sanity checks
    ASSERT_ERR(node != NULL);
    ASSERT_ERR(((uint32_t)mem_ptr > (uint32_t)node));

    prev_node = NULL;

    while(node != NULL)
    {
        ASSERT_ERR(node->corrupt_check == KERNEL_LIST_PATTERN);
        // check if the freed block is right after the current block
        if ((uint32_t)freed == ((uint32_t)node + node->free_size))
        {
            // append the freed block to the current one
            node->free_size += size;

            // check if this merge made the link between free blocks
            if (((uint32_t) node->next) == (((uint32_t)node) + node->free_size))
            {
                next_node = node->next;
                // add the size of the next node to the current node
                node->free_size += next_node->free_size;
                // update the next of the current node
                ASSERT_ERR(next_node != NULL);
                node->next = next_node->next;
                // update linked list.
                if(next_node->next != NULL)
                {
                    next_node->next->previous = node;
                }
            }
            goto free_end;
        }
        else if ((uint32_t)freed < (uint32_t)node)
        {
            // sanity check: can not happen before first node
            ASSERT_ERR(prev_node != NULL);

            // update the next pointer of the previous node
            prev_node->next = freed;
            freed->previous = prev_node;

            freed->corrupt_check = KERNEL_LIST_PATTERN;

            // check if the released node is right before the free block
            if (((uint32_t)freed + size) == (uint32_t)node)
            {
                // merge the two nodes
                freed->next = node->next;
                if(node->next != NULL)
                {
                    node->next->previous = freed;
                }
                freed->free_size = node->free_size + size;
            }
            else
            {
                // insert the new node
                freed->next = node;
                node->previous = freed;
                freed->free_size = size;
            }
            goto free_end;
        }

        // move to the next free block node
        prev_node = node;
        node = node->next;

    }

    freed->corrupt_check = KERNEL_LIST_PATTERN;

    // if reached here, freed block is after last free block and not contiguous
    prev_node->next = (struct mblock_free*)freed;
    freed->next = NULL;
    freed->previous = prev_node;
    freed->free_size = size;
    freed->corrupt_check = KERNEL_LIST_PATTERN;


free_end:
    #if KERNEL_PROFILING
    kernel_env.heap_used[cursor] -= size;
    #endif //KERNEL_PROFILING
    // end of protection
    GLOBAL_INT_RES();
}


bool kernel_is_free(void* mem_ptr)
{
    // use corrupt check info in order to know if pointer already free.
    return ((((struct mblock_used *)mem_ptr) - 1)->corrupt_check != KERNEL_ALLOCATED_PATTERN);
}

#if (KERNEL_PROFILING)
uint16_t kernel_get_mem_usage(uint8_t type)
{
    ASSERT_ERR(type < KERNEL_MEM_BLOCK_MAX);

    return kernel_env.heap_used[type];
}

uint32_t kernel_get_max_mem_usage(void)
{
    uint32_t ret = kernel_env.max_heap_used;

    // Cleared max measured value.
    kernel_env.max_heap_used = 0;

    return ret;
}
#else
uint16_t kernel_get_mem_usage(uint8_t type)
{ 
    return 0;
}

uint32_t kernel_get_max_mem_usage(void)
{
  
    return 0;
}
#endif // (KERNEL_PROFILING)

///@} KERNEL_MEM
