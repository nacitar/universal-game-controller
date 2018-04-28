#ifndef INCLUDED_UGC_INPUT_STATE_H_
#define INCLUDED_UGC_INPUT_STATE_H_

#include <linux/string.h>  // strcpy
#include <linux/slab.h>  // kmalloc, GFP_KERNEL, ...
#include <linux/types.h>  // uintptr_t
#include <linux/rbtree.h>

struct InputState {
  struct rb_node node;
  unsigned int type;
  unsigned int code;
  bool positive;
  // value not part of comparison
  __u32 value;
};
int InputState_Compare(const struct InputState *lhs,
    const struct InputState *rhs);

struct InputState *InputState_Search(struct rb_root *root,
    struct InputState *key);

bool InputState_Insert(struct rb_root *root, struct InputState *element);

// caller required to free element if necessary
void InputState_Erase(struct rb_root *root, struct InputState *element);

/*void InputState_Iterate(struct rb_root *root) {
  struct rb_node *node;
  for (node = rb_first(root); node; node = rb_next(node)) {
    printk(KERN_DEBUG pr_fmt("key=%u\n"), rb_entry(node, struct InputState, node)->key);
  }
}*/

// To replace an existing node in a tree with a new one with the same key, call:
//
//  void rb_replace_node(struct rb_node *old, struct rb_node *new,
//        struct rb_root *tree);
//
// Replacing a node this way does not re-sort the tree: If the new node doesn't
// have the same key as the old node, the rbtree will probably become corrupted.

#endif  // INCLUDED_UGC_INPUT_STATE_H_
