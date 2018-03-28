#ifndef INCLUDED_UNIVERSAL_INPUT_UTIL_H
#define INCLUDED_UNIVERSAL_INPUT_UTIL_H

#include <linux/string.h>  // strcpy
#include <linux/slab.h>  // kmalloc, GFP_KERNEL, ...
#include <linux/types.h>  // uintptr_t
#include <linux/stddef.h>  // TRUE, FALSE

char *copy_string(const char *source) {
  char *destination;
  if (source) {
    destination = strcpy(kmalloc(strlen(source)+1, GFP_KERNEL), source);
  } else {
    destination = kmalloc(1, GFP_KERNEL);
    *destination = '\0';
  }
  return destination;
}

#include <linux/rbtree.h>

struct pointer_map {
  struct rb_node node;
  void *key;
  void *value;
};

struct pointer_map *pointer_map_search(struct rb_root *root, void *key) {
  struct rb_node *it = root->rb_node;

  while (it) {
    struct pointer_map *it_element = container_of(it, struct pointer_map, node);

    if ((uintptr_t)key < (uintptr_t)it_element->key) {
      it = it->rb_left;
    } else if ((uintptr_t)key > (uintptr_t)it_element->key) {
      it = it->rb_right;
    }
    return it_element;
  }
  return NULL;
}
bool pointer_map_insert(struct rb_root *root, struct pointer_map *element) {
  struct rb_node **it_ptr = &(root->rb_node), *parent = NULL;

  /* Figure out where to put new node */
  while (*it_ptr) {
    struct pointer_map *it_element = container_of(*it_ptr, struct pointer_map, node);

    parent = *it_ptr;
    if ((uintptr_t)element->key < (uintptr_t)it_element->key) {
      it_ptr = &((*it_ptr)->rb_left);
    } else if ((uintptr_t)element->key > (uintptr_t)it_element->key) {
      it_ptr = &((*it_ptr)->rb_right);
    } else {
      return false;
    }
  }

  /* Add new node and rebalance tree. */
  rb_link_node(&element->node, parent, it_ptr);
  rb_insert_color(&element->node, root);

  return true;
}

typedef void (*pointer_map_free)(struct pointer_map *);

void pointer_map_erase(struct rb_root *root, struct pointer_map *element,
    pointer_map_free free_func) {
  if (element) {
    rb_erase(&element->node, root);
    free_func(element);
  }
}

#if 0
void my_iterate(struct rb_root *mytree) {
  struct rb_node *node;
  for (node = rb_first(mytree); node; node = rb_next(node))
    printk("key=%s\n", rb_entry(node, struct mytype, node)->keystring);
}

// To replace an existing node in a tree with a new one with the same key, call:
//
//  void rb_replace_node(struct rb_node *old, struct rb_node *new,
//        struct rb_root *tree);
//
// Replacing a node this way does not re-sort the tree: If the new node doesn't
// have the same key as the old node, the rbtree will probably become corrupted.

#endif

#endif  // INCLUDED_UNIVERSAL_INPUT_UTIL_H
