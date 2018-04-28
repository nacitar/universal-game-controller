#include <ugc/input_state.h>

int InputState_Compare(const struct InputState *lhs,
    const struct InputState *rhs) {
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

struct InputState *InputState_Search(struct rb_root *root,
    struct InputState *key) {
  struct rb_node *it = root->rb_node;
  while (it) {
    struct InputState *it_element = container_of(it, struct InputState, node);
    const int diff = InputState_Compare(key, it_element);
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
bool InputState_Insert(struct rb_root *root, struct InputState *element) {
  struct rb_node **it_ptr = &(root->rb_node), *parent = NULL;

  /* Figure out where to put new node */
  while (*it_ptr) {
    struct InputState *it_element = container_of(*it_ptr, struct InputState, node);
    const int diff = InputState_Compare(element, it_element);
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

void InputState_Erase(struct rb_root *root, struct InputState *element) {
  rb_erase(&element->node, root);
}
