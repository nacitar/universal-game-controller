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

struct uint_map {
  struct rb_node node;
  unsigned int key;
  unsigned int value;
};

struct uint_map *uint_map_search(struct rb_root *root, unsigned int key) {
  struct rb_node *it = root->rb_node;

  while (it) {
    struct uint_map *it_element = container_of(it, struct uint_map, node);

    if (key < it_element->key) {
      it = it->rb_left;
    } else if (key > it_element->key) {
      it = it->rb_right;
    }
    return it_element;
  }
  return NULL;
}
bool uint_map_insert(struct rb_root *root, struct uint_map *element) {
  struct rb_node **it_ptr = &(root->rb_node), *parent = NULL;

  /* Figure out where to put new node */
  while (*it_ptr) {
    struct uint_map *it_element = container_of(*it_ptr, struct uint_map, node);

    parent = *it_ptr;
    if (element->key < it_element->key) {
      it_ptr = &((*it_ptr)->rb_left);
    } else if (element->key > it_element->key) {
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

typedef void (*uint_map_free)(struct uint_map *);

// caller required to free element if necessary
void uint_map_erase(struct rb_root *root, struct uint_map *element) {
  rb_erase(&element->node, root);
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
