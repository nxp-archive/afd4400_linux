/*
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
 * Extended functionality of the device tree overlay feature.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *   Device Tree Overlay Extension
 *
 *   Overview
 *
 *   The device tree overlay feature allows the user to add nodes and
 *   add or modify existing properties.  The intented method to create an
 *   overlay node tree that can be inserted into the live tree during
 *   runtime is to create a dts source file following a special format.
 *   The dts file is compiled, using the device tree compiler (dtc),
 *   into a binary blob.  The blob is then inserted into the live tree
 *   during runtime by an overlay manager application.  Another method
 *   to insert the blob during runtime is to create a static C code array
 *   from the blob and use the overlay api insert it into the live tree.
 *
 *   Both of these methods required the user to engage in the compilation
 *   and tracking of the dts source and blob.  The original method
 *   requires the overlay blob be inserted through some manual means
 *   by the user.  In the latter method, it may be difficult for the user
 *   to keep track of the C code array to the dts source file as the
 *   C array is not in human readable form.
 *
 *   The overlay extension feature, from hereon refered to as overlay
 *   ext, provides another method to create the dts source and insertion.
 *   It allows the user to define the overlay node tree and properties
 *   in the code file in terms of data structures.  It provides a single
 *   api to compile the data structures and outputs a complete overlay
 *   node tree that can be used as an input to the overlay api which
 *   then inserts it into the live tree.  This feature does not require
 *   the use of the dtc.  The node and property contents are in human
 *   readable form in the code which eases development and debug.
 *
 *
 *   A Simple Example
 *
 *   A quick example is provided to illustrate the concept of the overlay
 *   ext feature.  Suppose a node for a real time clock (rtc) is to be created
 *   int the live tree under the existing node "i2cbus1".
 *
 *   -----------------------
 *   Dts overlay description:
 *
 *   /dts-v1/;
 *   /plugin/;
 *   {
 *      fragment@001 {
 *         target = <&i2cbus1>;
 *         __overlay__ {
 *             rtc:rtc@68{
 *                compatible = "dallas,ds3232";
 *                reg = <0x68>;
 *             };
 *         };
 *      };
 *   };
 *
 *   -----------------------
 *   Overlay ext description:
 *
 *   struct property rtc_property[] = {
 *     { .name = "name",       .value = "rtc",           .length = 1, ._flags = PROP_STR },
 *     { .name = "compatible", .value = "dallas,ds3232", .length = 1, ._flags = PROP_STR },
 *     { .name = "reg",        .value = "68",            .length = 1, ._flags = PROP_NUM },
 *     {}
 *   };
 *   struct device_node rtc_node[] = {
 *      { .name = "rtc", .full_name = "rtc@68", .properties = rtc_property, },
 *   };
 *   struct fragment_node fragment_rtc[] = {
 *      {
 *         .target = "&i2cbus1",
 *         .np_array = rtc_node,
 *         .ov_prop_array = NULL,
 *      }
 *   }
 *
 *
 *
 *   The overlay tree has the following hiearchy as shown below.  It
 *   has several standard nodes that are always created.  These nodes
 *   are:
 *
 *   __root__      (refered to as the root node)
 *   __symbols__   (refered to as the symbols node)
 *   __fixups__    (refered to as the fixup node)
 *
 *   The topmost node is the root node.  All other nodes are children
 *   of the root node.  The symbols node contains names of new node
 *   that the user has requested to be added.  The fixup node contains
 *   names of nodes being referenced by properties that the user has
 *   requested to be added or modified to existing properties.
 *
 *   A fragment node is created for each target node in the live tree
 *   that the user wants to add to or modify.  The __overlay__ is a
 *   standard child node of the fragment node.  Thus, the fragment
 *   node and child node are always created for each target.  The
 *   target node in the live tree is specified in a property in the
 *   fragment node.  The property has the member ->name = "target"
 *   and ->value is a 32-bit handle value pointing to the target node
 *   in the live tree.
 *
 *   Properties that the user wants to add to or modify in the target
 *   node are placed in the __overlay__ node.  New nodes to be added
 *   under the target node are added as child nodes of the __overlay__
 *   node.
 *
 *
 *    root {
 *
 *          fragment@001 {
 *
 *                targetnode1
 *
 *                __overlay__ {
 *
 *                     newprop1
 *                     newprop2
 *                     existingprop1
 *                     existingprop2
 *
 *                     newnodeA {
 *                           newpropA1
 *                           newpropA2
 *                           ...
 *                           newnodeAA {
 *                                 newpropAA1
 *                                 newpropAA2
 *                                 ...
 *                           }
 *                     }
 *                }
 *          }
 *
 *          fragment@002 {
 *
 *                target property
 *
 *                __overlay__ {
 *                      ....
 *                }
 *           }
 *       ....
 *
 *       __symbols__ {
 *
 *       }
 *       __fixups__ {
 *
 *       }
 *
 *    }
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/err.h>
#include "of_private.h"
#include <linux/overlay_ext.h>

#define MAX_STRING_SIZE	(128)

/* Value inserted for invalid node reference handle.  This is the
 * initial value for node references.  Once the reference is
 * resolved, it will have the proper node handle value.  Do not
 * change this value value as it is the same values used by the
 * device tree compiler.
 */
#define NODEREF_INVALID_VAL	0xdeadbeef

/* Little endian to big endian swap */
#define swap32(x) (\
	((x & 0x000000ff)<<24) | \
	((x & 0x0000ff00)<<8) | \
	((x & 0x00ff0000)>>8) | \
	((x & 0xff000000)>>24))

#define free_property(prop) {\
		kfree(prop->name);\
		kfree(prop->value);\
		kfree(prop);\
	}

/* Search for '&' that indicates a node reference.
 * This char must appear as the first char in the node
 * ->value string.
 * @returns
 *   - 1 char is '&'
 *   - 0 all other cases
 */
#define is_node_ref(ch) ((ch == '&') ? 1 : 0)

/* Search for '@' that indicates the node address.
 *
 * @returns
 *   - 1 string contains '@'
 *   - 0 string does not contain '@'
 * */
#define is_node_addr(str) ((strchr(str) != NULL) ? 1 : 0)

/* Print a u8 buffer in hex numbers */
#define print_buf(buf, num) {\
	int zzz = 0;\
	for (zzz = 0; zzz < num; ++zzz)\
		printk(KERN_INFO "%02x ", buf[zzz]);\
	printk("\n");\
}

/* Basic overlay node names/paths */
#define ROOT_NODE_NAME		""
#define ROOT_NODE_FULLNAME	"/"

#define SYMBOLS_NODE_NAME	"__symbols__"
#define SYMBOLS_NODE_FULLNAME	"/__symbols__"

#define FIXUPS_NODE_NAME	"__fixups__"
#define FIXUPS_NODE_FULLNAME	"/__fixups__"


/* Basic fragment node names/paths */
#define FRAGMENT_NODE_NAME	"fragment"
#define FRAGMENT_NODE_FULLNAME	"/fragment@000"

#define OVERLAY_NODE_NAME	"__overlay__"
#define OVERLAY_NODE_FULLNAME	"/fragment@000/__overlay__"


/* Standard nodes and properties for fragment, overlay, symbols,
 * and fixup nodes.
 *
 * ************ DO NOT MODIFY THESE VALUES. **************
 */
struct property frag_target_raw_prop[] = {
	{ .name = "target", .value = "deadbeef",
		.length = 1, ._flags = PROP_NUM },
	{ .name = "name", .value = "fragment",
		.length = 1, ._flags = PROP_STR},
	{} /* Null terminated */
};

struct property overlay_raw_prop[] = {
	{ .name = "name", .value = "__overlay__",
		.length = 1, ._flags = PROP_STR },
	{} /* Null terminated */
};

struct property symbols_raw_prop[] = {
	{ .name = "name", .value = "__symbols__",
		.length = 1, ._flags = PROP_STR},
	{} /* Null terminated */
};

struct property fixups_raw_prop[] = {
	{ .name = "name", .value = "__fixups__",
		.length = 1, ._flags = PROP_STR},
	{} /* Null terminated */
};

struct property node_linuxphandle_raw_prop[] = {
	{ .name = "linux,phandle", .value = "0",
		.length = 1, ._flags = PROP_NUM },
	{ .name = "phandle", .value = "0",
		.length = 1, ._flags = PROP_NUM },
	{} /* Null terminated */
};

static int search_addr_char(char *str);
static int of_add_node_name(struct device_node *np, char *name);
static struct device_node *ov_create_node(
	struct device_node *np_clone,
	char *full_name, char *name,
	struct device_node *np_symbols,
	struct device_node *np_fixups);
static int ov_add_properties_array(struct device_node *np,
	struct property *raw_prop_array);
static int add_noderef_to_fixup(struct device_node *np,
	struct device_node *np_fixups);

static int ov_add_multi_string_prop(struct device_node *np,
	char *prop_name, char *prop_value_str);
static struct device_node *ov_build_fragment_node(
	struct fragment_node *fnp,
	struct device_node *np_symbols,
	struct device_node *np_fixups,
	int index);
static int multistr_parse(char *src, char *dest, char delimiter,
	int maxstr, int *actualsize);
static struct property *process_raw_properties(struct property *rawprop);
static int multihex_parse(char *src, char *dest, int numhex, char delimiter);
static int find_num_multistr(char *src, int srcsize, char delimiter);
static int ov_update_node_phandles(struct device_node *np, u32 handle_value);

/*
 * free_nodes - Free memory resources allocated to a node and all
 *    of its children and siblings.  This includes properties of
 *    each node.  This function is recursive.
 *
 * @param np_root  Root node of the node tree.
 *
 * @returns
 *  -  On Success, zero value.
 *  -  On Failure, negative value indicating the error.
 */
int free_nodes(struct device_node *np_root)
{
	int ret = 0;
	struct device_node *child;

	if (!np_root)
		return -EINVAL;

	for_each_child_of_node(np_root, child) {
		free_nodes(child);
	}
	of_node_set_flag(np_root, OF_DETACHED);
	of_node_release(&np_root->kref);
	return ret;
}

/**
 * ov_update_node_phandles - traverse the node tree and increment
 *    the value of properties "linux,phandle" and "phandle" by one.
 *    This pair of properties exists for nodes **with addresses**
 *    in the tree that the user has defined.  Example of nodes with
 *    and without addresses in dts code are:
 *
 *    With address:
 *        i2cbus1: i2cbus1@A0000 { };
 *
 *    Without address:
 *        i2cbus1 { };
 *
 *    The "linux,phandle" and "phandle" property pair do not exists
 *    for nodes without an address.  Standard nodes created for the
 *    overlay tree: __overlay__, __symbols__, and __fixups__ do not
 *    these properties.
 *
 *    The update step is normally performed when the overlay tree
 *    creation is done with parent/child/next link complete. This
 *    function is recursive and traverses all nodes and childs in
 *    starting at the given node.
 *
 * @param np  An existing node to search
 * @param handle_value The starting handle value that is assigned to
 *    the first pair of "linux,phandle" and "phandles" property found.
 *    The value is then incremented.
 *
 * @returns
 *  -  On Success, 0
 *  -  On Failure, negative error number
 */
static int ov_update_node_phandles(struct device_node *np, u32 handle_value)
{
	int hval = handle_value;
	struct device_node *child;
	struct property *prop;
	u32 value;
	int updated = 0;

	for_each_property_of_node(np, prop) {
		if (strcmp(prop->name, "linux,phandle") == 0) {
			if (!of_property_read_u32(np, "linux,phandle",
				&value)) {
				*(u32 *)prop->value = swap32(hval);
				updated = 1;
			}
		}
		if (strcmp(prop->name, "phandle") == 0) {
			if (!of_property_read_u32(np, "phandle", &value))
				*(u32 *)prop->value = swap32(hval);
		}
	}

	if (updated)
		hval += 1;

	/* Recursively find all nodes in tree */
	for_each_child_of_node(np, child) {
		hval = ov_update_node_phandles(child, hval);
	}

	return hval;
}

/**
 * search_addr_char - Find the address char in a string.  The purpose
 *    is to determine if the node's full name contains an address
 *    indicated by the '@' char.
 *
 *    Example of node with an address.  Address is denote with the '@':
 *
 *	Dts syntax:
 *		test1: test1@123 {
 *		...
 *		}
 *
 *	Resulting node full name:
 *		/fragment@0/__overlay__/test1@123
 *
 *     Search is done backwards starting at the end of the string to look
 *     for the '@' BEFORE the '/' char.  If we find the '@' before the '/'
 *     char, then the node has an address.  In this case, the '@' appears
 *     first (test@123) and we determine the node has an address.
 *
 *    Example of node WITHOUT an address:
 *
 *	Dts syntax:
 *		test2 {
 *		...
 *		}
 *
 *	Resulting node full name:
 *		/fragment@0/__overlay__/test1
 *
 *      In this case in searching backwards, we run into the '/' first
 *	(test1) and determine that the node does NOT have an address.
 *
 *
 * @param str  Typically this is the node's full name string.
 *
 * @returns
 *  -  On Success, 0 (the addr char is found and node has an address)
 *  -  On Failure, -1
 */
static int search_addr_char(char *str)
{
	int len = strlen(str);

	if (len <= 0)
		return -1;

	/* Look for '@' and '/' char backwards in string.
	 * Return result of whichever char we hit first.
	 */
	while (len) {
		if (str[len-1] == '@')
			/* Addr char found first */
			return 0;
		if (str[len-1] == '/')
			/* Path delimiter found first */
			return -1;
		--len;
	}

	return -1;
}

/**
 * of_add_node_name - Add a string to the node's ->name member.
 *    The node name must be null initially before the new name
 *    string is added.
 *
 *    NOTE: The node->name structure member is not to be confused
 *    with a property with prop->name = "name".
 *
 * @param np  An existing node.  The ->name member must be null
 *    terminated.
 * @param name Node name to create new node with.
 *
 * @returns
 *  -  On Success, 0
 *  -  On Failure, negative error number
 */
static int of_add_node_name(struct device_node *np, char *name)
{
	int len;

	if (!np)
		return -EINVAL;

	/* Name must be null */
	if (np->name)
		return -EINVAL;

	len = strlen(name);
	if (len > 0) {
		np->name = kstrdup(name, GFP_KERNEL);
		if (np->name == NULL)
			return -ENOMEM;
	}
	return 0;
}

/**
 * get_str_pos - Given a string with sub-strings that are delineated
 *    a delimiter character, find the given index of the sub-string
 *    and return the position of that sub-string.
 *
 *    Index = 0 always returns a pointer to the beginning of str.
 *    Negative index value returns NULL.
 *
 *    Example 1:
 *       Input
 *           str = " xcvr    eeprom"
 *           delimiter = ' ' (space or 0x20 char)
 *
 *       Output with index = 0
 *           Char pointer to str[0] (start of " xcvr")
 *       Output with index = 1
 *           Char pointer to str[5] (start of "    eeprom")
 *
 *
 *    Example 2:
 *       Input
 *           str = "&gpioA \n 10 \n11 \n &gpioC \n 12 \n13"
 *           delimiter = '\n' (newline or 0x0a char)
 *
 *       Output with index = 0
 *           Char pointer to str[0] (start of "&gpioA")
 *       Output with index = 1
 *           Char pointer to str[8] (start of " 10")
 *       Output with index = 2
 *           Char pointer to str[13] (start of "11 ")
 *       Output with index = 3
 *           Char pointer to str[17] (start of " &gpioC")
 *       Output with index = 4
 *           Char pointer to str[26] (start of " 12 ")
 *       Output with index = 5
 *           Char pointer to str[31] (start of "13")
 *
 * @param str A string with sub-strings delineated by the delimiter char
 * @param index The index of the sub-string to search for
 * @param delimiter A character that is used to separate each of the
 *     sub-string(s).
 *
 * @returns
 *  -  On Success, pointer to the position in str that is the start of
 *     the sub-string with the given index.
 *  - On Failure, NULL pointer value.
 */
static char *get_str_pos(char *str, int index, char delimiter)
{
	int i = 0;
	char *strpos = str;

	if (index < 0)
		return NULL;

	if (index == 0)
		return str;

	while (i != index) {
		while (*strpos != delimiter)
			++strpos;
		/* Skip pass the delimiter */
		++strpos;
		++i;
	}
	return strpos;
}

/**
 * add_noderef_to_fixup - Search through a node's property list and
 *     find properties with node references.  For each property that
 *     contains a node reference, put the node reference in the
 *     __fixups__ node with the full path to the node, and create
 *     32-bit values for the node reference and any associated parameters.
 *
 *  property->value added to the __fixups__ node has the following format:
 *
 *     /fragment@001/__overlay__/nodename@1:propname:0
 *
 *     where "propname" is the name of the property that has the reference
 *     to a node.
 *
 *
 *  For example:
 *
 *      dts node syntax:
 *
 *         testnode: testnode@1 {
 *	      eeprom-i2c =<&i2cbus10 1 2>;
 *            ...
 *         };
 *
 *   Node property before fixup modifications:
 *      ->name: "eeprom-i2c"
 *      ->value: "&i2cbus10", "1", "2" (3 strings each separated by null chars)
 *	->length: 13
 *
 *   Property added to the __fixups__ node:
 *      ->name: "i2cbus10"
 *      ->value: "/fragment@001/__overlay__/testnode@1:eeprom-i2c:0"
 *
 *   Node property after fixup modifications:
 *      ->name: "eeprom-i2c"
 *      ->value: deadbeef0000000100000002 (3 32-bit values)
 *	->length: 12
 *
 *
 *  These steps are performed:
 *
 *  1) Properties ->value string is checked for a leading '&' char
 *    indicating a node reference.
 *
 *  2) Once found, the node reference is added to the fixup node as
 *    a property.  The property->name is the node reference.  The
 *    property->value is the full node path of the node that contains
 *    the node reference.  See description above.
 *
 *  3) The ->value pointer is pointed to a 32-bit value with the
 *    value of 0xDEADBEEF (which is the value used by device tree
 *    compiler).
 *
 *  4) The string pointed by ->value is freed.
 *
 * @param np  An existing node.  Its properties list are searched for
 *    any node references.
 * @param np_fixups An existing fixup node.  Node references found are
 *    put in the fixup node as properties.
 *
 * @returns
 *  -  Number of node references found that was added to fixup node.
 */
static int add_noderef_to_fixup(struct device_node *np,
	struct device_node *np_fixups)
{
	int ret = 0;
	struct property *prop;
	char *str;
	char *nextstr;
	char value_str[MAX_STRING_SIZE];
	u32 *numbuf32;
	int i, numstr, size;
	unsigned int num32;

	for_each_property_of_node(np, prop) {

		str = (char *)prop->value;

		if (!((is_node_ref(str[0])) && (strlen(str) > 1)))
			continue;

		/* Find the number of substrings */
		numstr = find_num_multistr(str, prop->length, '\0');

		/* The node references and their parameters are each
		 * turned into a 32-bit value.
		 */
		size = 4 * numstr;
		numbuf32 = kzalloc(size, GFP_KERNEL);

		i = 0;
		nextstr = str;
		while(i < numstr) {
			/* Find sub-string pos at given index */
			nextstr = get_str_pos(str, i, '\0');

			if (is_node_ref(nextstr[0])) {

				/* Node path and property name */
				memset(value_str, 0, MAX_STRING_SIZE);
				sprintf(value_str, "%s:%s:%i",
					(char *)np->full_name,
					prop->name,
					i * 4);/* Byte offset into numbuf32 */

				/* Add path to fixup node */
				ov_add_multi_string_prop(np_fixups,
					&nextstr[1], /* Skip leading '&' */
					value_str);

				numbuf32[i] = swap32(NODEREF_INVALID_VAL);

			} else {

				if (sscanf(nextstr, "%x", &num32) == 1)
					numbuf32[i] = swap32(num32);
			}
			++i;
		}

		/* Don't need the original buffer anymore */
		kfree(prop->value);

		prop->value = numbuf32;
		prop->length = size;
	}
	return ret;
}

/**
 * ov_create_node - Create a device tree node with full path name and
 *    node name.  If an existing node is passed in, its properties is
 *    cloned in the new node.
 *
 *    Note that this function was written specifically for the overlay
 *    extended feature.  The properties that is part of the np_clone,
 *    if any, are expected to be in a raw format, and the raw format
 *    is processed before the property is added to the newly created
 *    node.
 *
 *    This function is recursive.  If cloning from an existing node
 *    and np_clone->child is NOT null, the function calls on itself to
 *    create the child node and so on.
 *
 *    These functions are conditionally invoked to create the new node:
 *
 *       __of_node_dup()
 *       ov_add_properties_array()
 *       of_overlay_addto_symbols()
 *       ov_add_properties_array()
 *       add_noderef_to_fixup()
 *
 *
 * @param np_clone  An existing node to clone from include its properties.
 *     The clone node ->properties must point to a list of *raw* properties
 *     to create for the new node.  Set to null if the clone node does not
 *     have properties.
 *
 * @param full_name Node path name to create new node with.  Set to null to
 *     use the np_clone's node ->full_name.
 *
 * @param name Node name string to create new node with.  Set to null to
 *     use the np_clone's node ->name.
 *
 * @param np_symbols An existing symbols node.  If the new node has the
 *      '@' in its full name, that node full name is added to the symbols
 *      node as a property.  If this parameter is NULL, the node full
 *      name is not added.
 *
 * @param np_fixups An existing fixup node.  If the new node's properties
 *      references another node, the referenced node is added to the
 *      fixup node as a property.
 *
 * @returns
 *  -  On Success, handle to the new node
 *  -  On Failure, NULL
 */
static struct device_node *ov_create_node(
	struct device_node *np_clone,
	char *full_name, char *name,
	struct device_node *np_symbols,
	struct device_node *np_fixups)
{
	struct device_node *np = NULL;
	struct device_node *child = NULL;
	struct property *raw_prop_array = NULL;
	char *child_full_name;
	char *_full_name, *_name;
	int len;

	/* Default name and full name is given values */
	_name = (char *)name;
	_full_name = (char *)full_name;

	if (np_clone) {
		/* If names are not given, use the clone node's names */
		if (!full_name)
			_full_name = (char *)np_clone->full_name;
		if (!name)
			_name = (char *)np_clone->name;
	}

	if (np_clone) {
		if (np_clone->properties) {
			/* Save raw property array pointer and set node
			 * property ptr to null so that __of_node_dup()
			 * does not try to create them.  These raw
			 * properties need to be processed first which
			 * is done in ov_add_properties_array().
			 */
			raw_prop_array = np_clone->properties;
			np_clone->properties = NULL;
		}
	}

	/* Create the node.  Note that __of_node_dup() will try to
         * create all the properties that is pointed to by
         * np_clone->properties.  It will follow the property->next
         * linked list and create all of the properties in that order.
         *
         * Since this function expects the properties to be in a raw
         * raw format, np_clone->properties must be set to null.
         */
	np = __of_node_dup(np_clone, _full_name);
	if (!np) {
		pr_err("%s: Failed to create node name %s\n", __func__,
			_full_name);
		goto err;
	}

	/* Add the node name because __of_node_dup() does not do it */
	if (of_add_node_name(np, _name)) {
		pr_err("%s: Failed to add node name %s\n", __func__,
			_name);
		goto err;
	}

	/* Add properties for this node.  This is where the raw properties
         * are processed and aded to the new node.
         */
	if (raw_prop_array) {
		ov_add_properties_array(np, raw_prop_array);
		np_clone->properties = raw_prop_array;
	}

	/* If node has an address, add it to the symbols node as property
	 * and add two additional properties needed for this type of nodes.
	 */
	if (np_symbols) {
		/* Look for the '@' char to see if node has address */
		if (search_addr_char(_full_name) == 0) {
			/* Add node's full name to the symbols node as
			 * property.
			 */
			of_overlay_addto_symbols(np_symbols, np);

			/* Because this node has address, it needs two
			 * properties: "linux,phandle" and "phandle".  The
			 * property value for these properties are 32-bit
			 * unsigned int.
			 */
			ov_add_properties_array(np, node_linuxphandle_raw_prop);
		}
	}

	/* Find node references in the property list and add to fixup node */
	add_noderef_to_fixup(np, np_fixups);

	/* If not cloning from existing node, then we are done */
	if (!np_clone)
		goto out;

	/* Create child node in the list if it exists. */
	child = np_clone->child;
	if (child) {

		/* Create the full name for child node.  Child full name
		 * is created by taking the parent's (np_clone) full name
		 * and adding the full name of the child's full name.
		 * This should result in the full node path string.
		 */

		/* Temporarily create buffer to store the full name.
                 * +1 for '/' and +1 for null terminator
                 */
		len = strlen(_full_name) + strlen(child->full_name) + 1 + 1;
		child_full_name = kzalloc(len, GFP_KERNEL);

		if (!child_full_name) {
			pr_err("%s: Failed to alloc memory for node full name %s\n",
				__func__, _full_name);
			goto err;
		}
		sprintf(child_full_name, "%s/%s", _full_name, child->full_name);

		/* Recursive call */
		child = ov_create_node(child, child_full_name, NULL, np_symbols,
			np_fixups);
		kfree(child_full_name);

		if (child) {
			/* Connect the nodes */
			np->child = child;
			child->parent = np;
			np->allnext = child;
		} else {
			pr_err("%s: Failed to create child node %s\n", __func__,
				name);
			goto err;
		}
	}
out:
	return np;
err:
	if (np)
		of_node_release(&np->kref);
	return NULL;
}

/**
 * multistr_parse - Parses a string with sub-string(s) delineated by
 *    space(s).  Count the number of sub-strings found and return it.
 *    If a destination buffer is passed in, put each sub-string into
 *    destination buffer with each sub-string delineated by null (zero).
 *
 *    When a sub-string is found, it is stripped of leading and
 *    trailing white spaces (defined as space and tab characters) before
 *    being copied over to the destination buffer.  Whitespaces
 *    in between non-delimiter chars are preserved.  For example, a
 *    raw string "  Freescale, Inc  " results in "Freescale, Inc".
 *
 *    This function was designed to parse raw property->value buffer
 *    and copied the sub-strings found to the dest buffer in which the
 *    sub-strings are stripped of leading/trailing whitespaces.  Each
 *    sub-string are separated by a null char in the dest buffer.
 *
 *
 *    Example 1:
 *       Input:
 *           scr = "str1\0"
 *           maxstr = 1
 *           delimiter = ' ' (space, 0x20)
 *       Output:
 *           dest = "str1\0"
 *           actualsize = 5 (includes null terminator)
 *           return value = 1
 *
 *    Example 2:
 *       Input:
 *           scr = "str 1\n  str  2\n  str   3  \n  str 4\0"
 *           maxstr = 3
 *           delimiter = '\n' (newline, 0x0a)
 *       Output:
 *           dest = "str 1\0str  2\0str   3\0"
 *           actualsize = 21 (includes null terminators)
 *           return value = 3 (3 of 4 sub-strings found/copied,
 *                             "str4" is skipped)
 *
 *     If the sub-strings copied to the dest buffer exceeds destsize,
 *     the function returns with its current state of number of sub-strings
 *     copied and dest buffer with data already copied over from src buffer.
 *
 * @param src Source string containing one or more sub-strings delineated
 *      by space(s).  The string must be null terminated.
 *
 * @param dest Destination output string containing the sub-strings found
 *      in the source string delineated by null (zero) value.
 *
 * @param delimiter The character that separates the sub-strings.
 *
 * @param maxstr The maximum number of sub-strings to scan for in the
 *      source string.
 *
 * @param *actualsize Value returned indicating the total size of all of
 *      the sub-strings found in the source string and copied to the
 *      destination string.
 *
 * @returns
 *  -  On Success, zero or positive value indicating the number of
 *      sub-strings found in source string and copied to dest string.
 *  -  On Failure, negative value indicating the error.
 */
static int multistr_parse(char *src, char *dest, char delimiter, int maxstr,
	int *actualsize)
{
	int size, totalsize;
	int cnt;
	int i;
	char *str = src;
	char *strdest = NULL;
	int pos1, pos2, pos3;
	int max;

	max = 0;
	pos1 = pos2 = pos3 = 0;
	totalsize = cnt = 0;
	size = 0;

	/* Count total chars in string */
	while (str[max] != '\0')
		++max;

	if (dest)
		strdest = dest;

	while ((pos2 < max) && (cnt < maxstr)) {
		/* Skip leading white spaces */
		pos1 = pos2;
		while (1) {
			if ((str[pos1] == ' ') || (str[pos1] == '\t'))
				++pos1;
			else
				break;
		}
		pos2 = pos1;

		/* Search for delimiter */
		while ((str[pos2] != delimiter) && (str[pos2] != '\0'))
			++pos2;

		/* Save position of delimiter */
		pos3 = pos2;

		if (str[pos3] == '\0')
			--pos3;

		/* Skip continous delimiters and white spaces */
		while ((str[pos2] == ' ') || (str[pos2] == '\t') ||
			(str[pos2] == delimiter))
			++pos2;

		/* Skip trailing white space */
		while (pos3 > pos1) {
			if ((str[pos3] == ' ') || (str[pos3] == '\t') ||
				(str[pos3] == delimiter))
				--pos3;
			else
				break;
		}

		/* Size of sub-string, total size */
		size = pos3-pos1+1;

		totalsize += (size + 1); /* +1 for null inserted in dest str */

		/* Copy sub-string to destination and null-terminate if
		 * requested.
		 */
		if (dest) {
			char *strsrc;
			i = 0;
			strsrc = &str[pos1];
			while (i++ < size)
				*strdest++ = *strsrc++;
			*strdest++ = 0; /* Null terminate */
		}

		/* Continue only if a sub-string was found */
		if (size)
			++cnt;
		else
			break;
	}

	if (actualsize)
		*actualsize = totalsize;
	return cnt;
}

/*
 * find_num_multistr - Parses the source string containing sub-strings
 *      that are delineated by the given delimiter char.  Return the
 *      number of sub-strings found.
 *
 *      This function was designed to count the number of null terminated
 *      strings in the property->value buffer with delimiter char = '\0'.
 *
 *    Example 1:
 *       Input:
 *           scr = "gpioA\0"
 *           srcsize = 6
 *           delimiter = '\0'
 *       Output:
 *           return value = 1
 *
 *    Example 2:
 *       Input:
 *           scr = "  gpioA  \0\0\0 gpio  B  \0"
 *           srcsize = 23
 *           delimiter = '\0'
 *       Output:
 *           return value = 2
 *
 * @param src Source string containing one or more sub-strings delineated
 *      by null char(s).  Each sub-string is null terminated.
 *
 * @param srcsize Total size of the string which is the size of all of
 *      the substrings plus nulls or leading and intermediate whitespaces.
 *
 * @returns
 *  -  On Success, a value indicating the number of sub-strings found in
 *     source string.
 *  -  On Failure, negative value indicating the error.
 */
static int find_num_multistr(char *src, int srcsize, char delimiter)
{
	int cnt;
	int size, actual;
	char *tmpbuf;
	unsigned int pos1, pos2;

	if (!src)
		return -EINVAL;

	if (!srcsize)
		size = strlen(src);
	else
		size = srcsize;

	tmpbuf = kzalloc(size, GFP_KERNEL);
	pos1 = pos2 = actual = cnt = 0;
	do {
		memset(tmpbuf, 0, size);
		if (sscanf(&src[pos1], "%s%n", tmpbuf, &pos2) == 1) {
			actual += strlen(tmpbuf);
			pos1 += pos2;
			++cnt;

			/* Skip pass delimiter(s) */
			while (pos1 < size) {
				if (src[pos1] == delimiter)
					++pos1;
				else
					break;
			}
		} else
			break;
	} while (pos1 < size);

	kfree(tmpbuf);

	/* Number of substrings found */
	return cnt;
}

/*
 * multihex_parse - Parses a string containing hex numbers delineated
 *     by the given delimiter. Put each 32-bit hex number into destination
 *     buffer in continous fashion.
 *
 *     This function was designed to parse raw property->value buffer
 *     to search for sub-strings that are hex values delineated by the
 *     newline char '\n'.
 *
 *
 *    Example:
 *       Input:
 *           scr = "23  a6  fbb0 "
 *           delimiter = '\n'
 *           numhex = 3
 *       Output:
 *           dest[] = 00000023000000a60000fbb0 (3 32-bit, 12 bytes)
 *           return value = 3
 *
 * @param src Source string containing one or more hex number delineated
 *      by space(s).  The string is null terminated.  Hex numbers in
 *      the string must be as-is: "23 a6 fb" without any prefix.
 *
 * @param dest Destination output buffer containing the 32-bit values
 *      continously (no delineation).
 *
 * @param numhex The number of hex numbers to scan for in the source string.
 *
 * @returns
 *  -  On Success, zero or positive value indicating the number of
 *      hex numbers found in source string, converted, and stored in
 *      dest buffer.
 *  -  On Failure, negative value indicating the error.
 */
static int multihex_parse(char *src, char *dest, int numhex, char delimiter)
{
	int i, cnt;
	unsigned int *numbuf, num32;
	unsigned int pos1, pos2;

	if ((!src) || (!dest) || (numhex == 0))
		return -EINVAL;

	numbuf = (unsigned int *)dest;

	/* Scan in all the numbers, each number is stored in 4-byte
	 * area in ->value buffer.
	 */
	pos1 = pos2 = cnt = 0;
	for (i = 0; i < numhex; ++i) {
		/* Skip null delimiter */
		while (src[pos1] == delimiter)
			++pos1;
		if (sscanf(&src[pos1], " %x%n", &num32, &pos2) == 1) {
			numbuf[i] = swap32(num32);
			pos1 += pos2;
			++cnt;
		}
	}

	/* Number of hex numbers processed */
	return cnt;
}

/**
 * process_raw_properties - Process raw properties which are required
 *    to have property members follow rules to help process the raw
 *    data.
 *
 *    In the description below, all strings mentioned must be null
 *    terminated.
 *
 *    struct property
 *       ->name - A string containing the name of the property.
 *       ->value - A string containing the raw property value.
 *          Sub-strings are delineated by the newline ('\n', 0x0a)
 *          character.
 *       ->length - The number of sub-strings in ->value buffer.
 *       ->_flags - Set to PROP_NUM if the ->value buffer contains
 *          sub-string(s) of hex numbers.  Set to PROP_STR if the
 *          ->value buffer contains sub-string(s) of text/words.
 *
 *    Example:
 *
 *       -------------------------------------
 *       Dts source (numbers are in decimal and hex):
 *
 *       #address-cells = <1>;
 *       platform-drv-name = "roc_misc_driver";
 *       interrupt-parent = <&gpioA>;
 *       interrupts = <16 0x04>, <17 0x04>;
 *       cs-gpios = <&gpioC 16 0>,
 *                  <&gpioC 18 0>;
 *       ipmi-mfg-str = "Analog Devices";
 *       ipmi-name-str = "9368-2R1a,9525R0,9368-1R1a",
 *            "9368-2R1a,9525,9368-1R1a";
 *
 *       -------------------------------------
 *      Translation to raw properties in statically defined structures.
 *      Each member of the example_prop[] array can be passed to this
 *      for processing.
 *
 *      NOTE: All numbers must be in hexidecimal without any prefix.
 *
 *   struct property example_prop[] = {
 *      { .name = "name", .value = "xcvr-roc1", .length = 1, ._flags = PROP_STR },
 *
 *      { .name = "#address-cells", .value = "1", .length = 1, ._flags = PROP_NUM },
 *
 *      { .name = "platform-drv-name", .value = "roc_misc_driver",
 *              .length = 1, ._flags = PROP_STR },
 *
 *      { .name = "interrupt-parent", .value = "&gpioA", .length = 1, ._flags = PROP_STR },
 *
 *      { .name = "interrupts", .value = "10\n 4\n 11\n 4", .length = 4, ._flags = PROP_NUM },
 *
 *      { .name = "cs-gpios", .value = "&gpioC \n 10 \n 0\n &gpioC \n 12 \n 0",
 *              .length = 9, ._flags = PROP_STR },
 *
 *      { .name = "ipmi-mfg-str", .value = "Analog Devices", .length = 1, ._flags = PROP_STR },
 *
 *      { .name = "ipmi-name-str", .value = "9368-2R1a,9525R0,9368-1R1a \n 9368-2R1a,9525,9368-1R1a",
 *   };
 *
 *       -------------------------------------
 *       Processing of the example_prop[] results in the following conversion
 *       at the end of the function.  The ->value buffer is deallocated and a
 *       new buffer is created with the processed values shown below.
 *
 *
 *   example_prop[0]
 *      ->name = "name\0"
 *      ->value = "xcvr-roc1\0"
 *      ->length = 10
 *
 *   example_prop[1]
 *      ->name = "#address-cells\0"
 *      ->value = 00000001
 *      ->length = 4
 *
 *   example_prop[2]
 *      ->name = "platform-drv-name\0"
 *      ->value = "roc_misc_driver\0"
 *      ->length = 16
 *
 *      NOTE: This property contains node reference(s) and will be further
 *      modified during the overlay tree creation *after* this function call.
 *      Eventually, the ->value buffer will be converted to a 32-bit value
 *      buffer to hold the node reference pointer and any parameters.
 *   example_prop[3]
 *      ->name = "interrupt-parent\0"
 *      ->value = "&gpioA\0"
 *      ->length = 7
 *
 *   example_prop[4]
 *      ->name = "interrupts\0"
 *      ->value = 000000010000000040000001100000004 (4 32-bit values)
 *      ->length = 16
 *
 *      NOTE: This property contains node reference(s) and will be further
 *      modified during the overlay tree creation *after* this function call.
 *      Eventually, the ->value buffer will be converted to a 32-bit value
 *      buffer to hold the node reference pointer and any parameters.
 *   example_prop[5]
 *      ->name = "cs-gpios\0"
 *      ->value = "&gpioC\010\00\0&gpioC\012\00\0"
 *      ->length = 23
 *
 *   example_prop[6]
 *      ->name = "ipmi-mfg-str\0"
 *      ->value = "Analog Devices\0"
 *      ->length = 15
 *
 *   example_prop[7]
 *      ->name = "ipmi-name-str\0"
 *      ->value = "9368-2R1a,9525R0,9368-1R1a\09368-2R1a,9525,9368-1R1a\0"
 *      ->length = 52
 *
 * @param rawprop Pointer to a property structure containing data in
 *    raw format.
 *
 * @returns
 *  -  On Success, Processed property structure ready to be added to a node.
 *  -  On Failure, NULL value.
 */
static struct property *process_raw_properties(struct property *rawprop)
{
	int ret = 0;
	int size, actual;
	char *strbuf;
	struct property *newprop;

	newprop = kzalloc(sizeof(struct property), GFP_KERNEL);
	if (!newprop)
		return NULL;
	newprop->name = kstrdup(rawprop->name, GFP_KERNEL);

	/* Preprocess the properties.
	 *
	 *  - For properties that are numbers, convert hex number string
	 *    to numbers.  This case includes an array of numbers.
	 *
	 *      rawprop->value points to a string of hex integer(s) delineated
	 *         by spaces.
	 *	rawprop->length indicates the number of hex integers (NOT the
	 *         number of bytes in the hex number strings).
	 *
	 *  - For properties that are strings, parse all the strings
	 *    (if more than one strings).
	 *
	 *    rawprop->value points a char buffer containing one or more
	 *       strings delineated by spaces.
	 *
	 *    rawprop->length indicates the number of strings in the
	 *       char buffer.
	 *
	 */
	switch (rawprop->_flags) {
	case PROP_NUM:
		/* Raw ->length is number of 32-bit numbers in property.
		 * Number if bytes is then = 4 * ->length.
		 */
		newprop->length = 4 * rawprop->length;
		newprop->value = kzalloc(newprop->length, GFP_KERNEL);

		ret = multihex_parse((char *)rawprop->value,
			(char *)newprop->value, rawprop->length, '\n');
		break;
	case PROP_STR:

		/* NOTE: String buffer is allocated with worst case size and
		 * assigned to ->value.  If the raw string contains sub-strings
		 * separated by 1 space, then worst case size is the actual
		 * size.  If separated by more than one spaces, then the
		 * actual size is a bit less than the worst case size.  We'll
		 * use worst case size since it could only be few bytes more
		 * than what we need.
		 *
		 * NOTE that this implies that the raw string pointed to by
		 * ->value must not have null ('\0') in between sub-strings.
		 */

		/* Worst case size */
		size = strlen(rawprop->value) + 1;

		/* Buffer to store parsed string(s) delineated by nulls */
		strbuf = kzalloc(size, GFP_KERNEL);
		newprop->value = (void *)strbuf;

		/* Parse string and save the actual length */
		actual = 0;
		ret = multistr_parse((char *)rawprop->value, strbuf, '\n',
			rawprop->length, &actual);

		if (ret > 0)
			newprop->length = actual;
		break;
	default:
		break;
	}

	if (ret <= 0)
		goto err;

	return newprop;
err:
	pr_err("%s: Can't create from raw property.\n", __func__);
	free_property(newprop);
	return NULL;
}

/*
 * ov_add_properties_array - Process an array of properties with
 *    property data in raw format.
 *
 * @param np Device node to add the properties to.
 *
 * @param raw_prop_array An array of property structures in raw format.
 *    The last element in the array must be null (all zeros).
 *    See process_raw_properties() for examples of raw format.
 *
 * @returns
 *  -  On Success, zero value.
 *  -  On Failure, negative value indicating the error.
 */
static int ov_add_properties_array(struct device_node *np,
	struct property *raw_prop_array)
{
	int ret = 0;
	int i = 0;
	struct property *tmpprop;
	struct property *newprop;

	if ((!np) || (!raw_prop_array))
		return -EINVAL;

	/* TODO: Check every property and verify that ->lenght is
	 * non-zero.  ->length in this case means the number of
	 * strings or 32-bit values in the ->value string.
	 */

	while (raw_prop_array[i].name != NULL) {

			/* Convert raw property first */
			tmpprop = process_raw_properties(&raw_prop_array[i]);
			if (tmpprop) {
				/* __of_prop_dup() checks for duplicate
				 * property and will fail if that is true.
				 */
				newprop = __of_prop_dup(tmpprop, GFP_KERNEL);
				free_property(tmpprop);
			} else
				return -EINVAL;

			/* If new prop created, then add it to node */
			if (newprop)
				ret = of_add_property(np, newprop);
			else
				return -EINVAL;

			/* If error, destroy property and quit */
			if (ret) {
				free_property(newprop);
				return ret;
			}
		++i;
	}
	return 0;
}

/**
 * ov_add_multi_string_prop - Add a property containing string data.
 *     If the property does not exists, create it and add the new
 *     string.  If the property exists and already contain string(s),
 *     add the new string to the property and preserve existing strings.
 *     All strings are null terminated.  The  property->value member
 *     points to the start of the string(s).
 *
 *     Example of existing property:
 *         property->name = "mfg\0"
 *         property->value = "Freescale\0"
 *
 *     New string to add:
 *         prop_value_str = "NXP\0"
 *
 *     New property value:
 *         property->value = "Freescale\0NXP\0"
 *
 * @param np  Pointer to node to add property to.
 * @param prop_name Property name string
 * @param prop_value_str New string to add.
 *
 * @returns
 *  -  On Success, 0
 *  -  On Failure, negative error code
 */
static int ov_add_multi_string_prop(struct device_node *np,
	char *prop_name, char *prop_value_str)
{
	int ret = 0;
	struct property *prop;
	int cur_len = 0;
	int len = 0;
	char *prop_new_value_str = NULL;

	if ((!np) || (!prop_name) || (!prop_value_str))
		return -EINVAL;

	/* Look for the property to see if it exists */
	prop = of_find_property(np, prop_name, &cur_len);

	if (!prop) {
		/* Property does not exist, create new one */
		prop = kzalloc(sizeof(struct property), GFP_KERNEL);
		if (!prop) {
			ret = -ENOMEM;
			goto err;
		}

		prop->name = kstrdup(prop_name, GFP_KERNEL);
		prop->value = kstrdup(prop_value_str, GFP_KERNEL);

		/* Note that kstrdup() added +1 char to string
		 * for null termination.
		 */
		prop->length = strlen(prop_value_str) + 1;

		ret = of_add_property(np, prop);
		if (ret)
			goto err;

	} else {
		int i;

		/* Property exists, move current string to new string
		 * and free the old string.
		 */

		/* Current len plus new string len with null terminator.
		 * Note that cur_len value came from the existing
		 * property so it should have counted the null
		 * terminator already.
		 */
		len = cur_len + (strlen(prop_value_str) + 1);
		len = cur_len + (strlen(prop_value_str) + 1);

		prop_new_value_str = kzalloc(len, GFP_KERNEL);
		if (!prop_new_value_str)
			return -ENOMEM;

		/* Copy over current string. Cannot use string functions
		 * for copying because they stop at null terminators.  The
		 * property string here have multiple strings with null
		 * termination in between.  So do it the old fashion way.
		 */
		for (i = 0; i < cur_len; ++i)
			prop_new_value_str[i] = ((char *)prop->value)[i];

		/* Add in the new string just after the null terminator
		 * of current string.
		 */
		strcpy(&prop_new_value_str[cur_len], prop_value_str);

		/* Free the old string and point to new string */
		kfree(prop->value);
		prop->value = (void *)prop_new_value_str;
		prop->length = len;
	}

	return ret;
err:
	/* Can't add property */
	free_property(prop);
	return ret;
}

/*
 * ov_build_fragment_node - Create a fragment node containing all the
 *    properties and node hiearchy as specified by the given fragment
 *    node structure.
 *
 *    For any properties that contain node reference(s), the node
 *    references are added to the __fixups__ node.  This is a requirement
 *    of the overlay feature so that when the fragmens are added to the
 *    live tree, node references can be resolved to point to the proper
 *    node.
 *
 * @param fnp  A fragment node containing links to node structure(s)
 *    and associated properties to be created.
 *
 * @returns
 *  -  On Success, handle to the fragment device node that is created.
 *  -  On Failure, NULL
 */
static struct device_node *ov_build_fragment_node(
	struct fragment_node *fnp,
	struct device_node *np_symbols,
	struct device_node *np_fixups,
	int index)
{
	struct device_node *np_fragment = NULL;
	struct device_node *np_overlay = NULL;
	struct device_node *np_child, *np_cur;
	char fragment_fullname[MAX_STRING_SIZE];
	char overlay_fullname[MAX_STRING_SIZE];
	char node_fullname[MAX_STRING_SIZE];
	int ret;
	int i;

	/* Fragment/overlay full names which includes the node name and
	 * path.
	 */
	memset(fragment_fullname, 0, MAX_STRING_SIZE);
	sprintf(fragment_fullname, "/%s@%03i", FRAGMENT_NODE_NAME, index);

	memset(overlay_fullname, 0, MAX_STRING_SIZE);
	sprintf(overlay_fullname, "%s/%s", fragment_fullname,
		OVERLAY_NODE_NAME);

	/* Create the basic fragment nodes */
	np_fragment = ov_create_node(NULL, fragment_fullname,
		FRAGMENT_NODE_NAME, NULL, NULL);
	np_overlay = ov_create_node(NULL, overlay_fullname,
		OVERLAY_NODE_NAME, NULL, NULL);

	if ((!np_fragment) || (!np_overlay))
		goto err;

	/* Create target and name property for fragment node */
	ov_add_properties_array(np_fragment, frag_target_raw_prop);

	/* Create name property for overlay node */
	ov_add_properties_array(np_overlay, overlay_raw_prop);

	/* Overlay node is a child of the fragment node */
	np_fragment->child = np_overlay;
	np_overlay->parent = np_fragment;

	/* Create properties for the overlay node, if any */
	if (fnp->ov_prop_array != NULL) {
		ret = ov_add_properties_array(np_overlay, fnp->ov_prop_array);
		if (ret)
			goto err;
	}
	/* Node references are added to fixup node */
	add_noderef_to_fixup(np_overlay, np_fixups);

	/* Create overlay child nodes and its properties for the
	 * target node, if any.
	 */
	if (!fnp->np_array)
		goto out; /* All done */

	/* TODO: Loop through all nodes and verify that ->name and
	 * ->full_name are not NULL.
	 */

	/* Create all of the child nodes in the array */
	i = 0;
	while (fnp->np_array[i].name != NULL) {

		/* Full name for the node is the node path:
		 * "/fragment@X/__overlay__/node_full_name"
		 */
		memset(node_fullname, 0, MAX_STRING_SIZE);
		sprintf(node_fullname, "%s/%s", overlay_fullname,
			fnp->np_array[i].full_name);

		np_child = ov_create_node(&fnp->np_array[i], node_fullname,
				NULL, np_symbols, np_fixups);

		if (np_child) {

			/* Update the node parent/child/next pointers */
			np_child->parent = np_overlay;
			if (i == 0) {
				np_overlay->child = np_child;
				np_overlay->next = np_child;
			} else {
				/* Current child node points to new child. */
				np_cur->sibling = np_child;
				np_cur->next = np_child;
			}
			np_cur = np_child;
		}
		++i;
	}
	np_fragment->next = np_fragment->child;
	np_fragment->allnext = np_fragment->child;

out:
	return np_fragment;
err:
	if (np_fragment)
		of_node_release(&np_fragment->kref);
	if (np_overlay)
		of_node_release(&np_overlay->kref);
	return NULL;
}

/*
 * ov_create_tree - Create an overlay device tree node.  The resulting
 *    root node is ready to be used to overlay the tree into the live
 *    tree using the of_overlay_create().
 *
 * @param fnp_array  An array of fragment nodes of the overlay tree.
 *    Note that this array must be null terminated like so:
 *
 *	struct fragment_node example[] = {
 *		{ .target = "test1", ...},
 *		{ .target = "test2", ...},
 *		{}
 *	};
 *
 * @returns
 *  -  On Success, handle to the overlay tree root node.
 *  -  On Failure, NULL
 */
struct device_node *ov_create_tree(struct fragment_node *fnp_array)
{
	struct device_node *np_root = NULL;
	struct device_node *np_symbols = NULL;
	struct device_node *np_fixups = NULL;
	struct device_node *np_fragment = NULL;
	struct device_node *np_fragment_cur = NULL;
	char prop_name[MAX_STRING_SIZE];
	char prop_value_str[MAX_STRING_SIZE];
	int i;
	char *str;

	/* Create the basic root, symbols, and fixup nodes */
	np_root = ov_create_node(NULL, ROOT_NODE_FULLNAME,
		ROOT_NODE_NAME, NULL, NULL);
	np_symbols = ov_create_node(NULL, SYMBOLS_NODE_FULLNAME,
		SYMBOLS_NODE_NAME, NULL, NULL);
	np_fixups = ov_create_node(NULL, FIXUPS_NODE_FULLNAME,
		FIXUPS_NODE_NAME, NULL, NULL);

	if ((!np_root) || (!np_symbols) || (!np_fixups))
		goto out;

	/* Create name property for symbols node */
	ov_add_properties_array(np_symbols, symbols_raw_prop);
	/* Create name property for fixups node */
	ov_add_properties_array(np_fixups, fixups_raw_prop);

	/* Root->next points to fixups */
	np_root->next = np_fixups;

	/* Make sure all fragment nodes have a target. */
	i = 0;
	while (fnp_array[i].target != NULL) {

		str = fnp_array[i].target;

		if  (!((is_node_ref(str[0])) && (strlen(str) > 1))) {
			pr_err("%s: Fragment index %i does not have valid target\n",
				__func__, i);
		}
		++i;
	}

	/* Create all the fragment nodes and any child nodes associated with
	 * each fragment.
	 */
	i = 0;
	while (fnp_array[i].target != NULL) {

		/* Create fragment node */
		np_fragment = ov_build_fragment_node(&fnp_array[i],
			np_symbols, np_fixups, i+1);

		if (np_fragment) {

			/* Set the target string as a property of the fixup
			 * node. The property for the target in the fixup
			 * node loos like:
			 *    ->name = "target_name_string"
			 *    ->value =
			 *             "/fragment@001:target:0",
			 *             "/fragment@002:target:0"
			 *
			 * The property value can contain several strings with
			 * each string for each fragment node that references
			 * it.
			 */
			memset(prop_name, 0, MAX_STRING_SIZE);
			memset(prop_value_str, 0, MAX_STRING_SIZE);

			strcpy(prop_name,
				&fnp_array[i].target[1]); /* Skip '&' */

			sprintf(prop_value_str, "%s:target:0",
				np_fragment->full_name);

			ov_add_multi_string_prop(np_fixups, prop_name,
				prop_value_str);

			/* Set the linked list pointers */
			np_fragment->parent = np_root;
			if (i == 0) {
				np_root->child = np_fragment;
				np_root->allnext = np_fragment;
			} else {
				np_fragment_cur->sibling = np_fragment;
			}
			np_fragment_cur = np_fragment;

			/* In case this is the last fragment node */
			np_fragment->sibling = np_symbols;
		}
		++i;
	}

	if (np_fragment)
		np_fragment->next = np_symbols;

	np_root->next = np_fixups;
	np_symbols->sibling = np_fixups;
	np_symbols->allnext = np_fixups;


	/* Finally, find all nodes with "linux,phandle" and "phandle"
	 * property and increment the values.
	 */
	ov_update_node_phandles(np_root, 1);

	return np_root;
out:
	if (np_root)
		of_node_release(&np_root->kref);
	if (np_symbols)
		of_node_release(&np_symbols->kref);
	if (np_fixups)
		of_node_release(&np_fixups->kref);

	return NULL;
}

/*
 * ov_load - Perform the steps necessary to create the overlay tree,
 *    resolve handles, and insert it into the live tree.  The overlay
 *    tree is freed when the process is complete.
 *
 * @param fnp_array  An array of fragment nodes of the overlay tree.
 *    Note that this array must be null terminated like so:
 *
 *	struct fragment_node example[] = {
 *		{ .target = "test1", ...},
 *		{ .target = "test2", ...},
 *		{}
 *	};
 *
 * @returns
 *  -  On Success, the ID of the overlay which is zero or greater.
 *  -  On Failure, negative value indicating the error.
 */
int ov_load(struct fragment_node *fnp_array)
{
	int ret = 0;
	struct device_node *np_ov_root;

	if (!fnp_array)
		return -EINVAL;

	/* Create the overlay tree from static structures */
	np_ov_root = ov_create_tree(fnp_array);
	if (!np_ov_root) {
		pr_err("\n%s: Failed to create overlay tree\n", __func__);
		return -EINVAL;
	}

	/* Must set detached flag */
	of_node_set_flag(np_ov_root, OF_DETACHED);

	/* Resolve node references */
	ret = of_resolve_phandles(np_ov_root);
	if (ret) {
		pr_err("%s: Failed to resolve overlay tree, err = %i\n",
			__func__, ret);
		goto out;
	}

	/* Apply the overlay tree to live tree.  Return value is the id of the
	 * overlay.
	 */
	ret = of_overlay_create(np_ov_root);
	if (ret < 0) {
		pr_err("%s: Failed to apply overlay, err = %i\n",
			__func__, ret);
		goto out;
	}
out:
	free_nodes(np_ov_root);
	return ret;
}
