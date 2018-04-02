#ifndef INCLUDED_UGC_INPUT_H
#define INCLUDED_UGC_INPUT_H

#include <linux/string.h>  // strcpy
#include <linux/slab.h>  // kmalloc, GFP_KERNEL, ...
#include <linux/types.h>  // uintptr_t
#include <linux/stddef.h>  // TRUE, FALSE
#include <linux/rbtree.h>

struct ugc_input {
  struct rb_node node;
  unsigned int type;
  unsigned int code;
  bool positive;
  // value not part of comparison
  unsigned int value;
};
int ugc_input_compare(const struct ugc_input *lhs,
    const struct ugc_input *rhs) {
  int diff;

  diff = lhs->type - rhs->type;
  if (diff) {
    return diff;
  }
  diff = lhs->code - rhs->code;
  if (diff) {
    return diff;
  }
  return (lhs->positive - lhs->positive);
}

struct ugc_input *ugc_input_search(struct rb_root *root, struct ugc_input* key) {
  struct rb_node *it = root->rb_node;
  while (it) {
    struct ugc_input *it_element = container_of(it, struct ugc_input, node);
    const int diff = ugc_input_compare(key, it_element);
    if (diff < 0) {
      it = it->rb_left;
    } else if (diff > 0) {
      it = it->rb_right;
    } else {
      return it_element;
    }
  }
  return NULL;
}
bool ugc_input_insert(struct rb_root *root, struct ugc_input *element) {
  struct rb_node **it_ptr = &(root->rb_node), *parent = NULL;

  /* Figure out where to put new node */
  while (*it_ptr) {
    struct ugc_input *it_element = container_of(*it_ptr, struct ugc_input, node);
    const int diff = ugc_input_compare(element, it_element);
    parent = *it_ptr;
    if (diff < 0) {
      it_ptr = &((*it_ptr)->rb_left);
    } else if (diff > 0) {
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

// caller required to free element if necessary
void ugc_input_erase(struct rb_root *root, struct ugc_input *element) {
  rb_erase(&element->node, root);
}

/*void ugc_input_iterate(struct rb_root *root) {
  struct rb_node *node;
  for (node = rb_first(root); node; node = rb_next(node)) {
    printk(KERN_DEBUG pr_fmt("key=%u\n"), rb_entry(node, struct ugc_input, node)->key);
  }
}*/

// To replace an existing node in a tree with a new one with the same key, call:
//
//  void rb_replace_node(struct rb_node *old, struct rb_node *new,
//        struct rb_root *tree);
//
// Replacing a node this way does not re-sort the tree: If the new node doesn't
// have the same key as the old node, the rbtree will probably become corrupted.


#endif  // INCLUDED_UGC_INPUT_H
