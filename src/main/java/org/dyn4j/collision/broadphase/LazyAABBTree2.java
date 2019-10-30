/*
 * Copyright (c) 2010-2017 William Bittle  http://www.dyn4j.org/
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted 
 * provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice, this list of conditions 
 *     and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
 *     and the following disclaimer in the documentation and/or other materials provided with the 
 *     distribution.
 *   * Neither the name of dyn4j nor the names of its contributors may be used to endorse or 
 *     promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.dyn4j.collision.broadphase;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.dyn4j.collision.Collidable;
import org.dyn4j.collision.Collisions;
import org.dyn4j.collision.Fixture;
import org.dyn4j.dynamics.World;
import org.dyn4j.geometry.AABB;
import org.dyn4j.geometry.Ray;
import org.dyn4j.geometry.Vector2;

/**
 * Implementation of a self-balancing axis-aligned bounding box tree broad-phase collision detection algorithm.
 * <p>
 * This class implements a aabb tree broad-phase detector that is based on ideas from {@link DynamicAABBTree} but with some very critical improvements.
 * This data structure is lazy in the sense that it will build the actual tree as late as possible (hence the name).
 * Performance is optimized for fast detection of collisions, as required by the {@link World} class. Raycasting and other functionalities should see no big improvements.
 * Insertion is O(1), update is O(logn) but batch update (update of all bodies) is O(n), remove is O(logn) average but O(n) worse.
 * <p>
 * The class will rebuild the whole tree at each detection and will detect the collisions at the same time in an efficient manner.
 * <p>
 * This structure keeps the bodies sorted by the radius of their fixtures and rebuilds the tree each time in order to construct better trees.
 * 
 * @author Manolis Tsamis
 * @param <E> the {@link Collidable} type
 * @param <T> the {@link Fixture} type
 * @version 3.3.1
 * @since 3.3.1
 */
public class LazyAABBTree2<E extends Collidable<T>, T extends Fixture> extends AbstractBroadphaseDetector<E, T> implements BatchBroadphaseDetector<E, T> {
	/** The root node of the tree */
	LazyAABBTreeNode2 root;
	
	/** Id to leaf map for fast lookup in tree of list */
	final Map<BroadphaseKey, LazyAABBTreeLeaf2<E, T>> elementMap;
	
	/** List of all leafs, either on tree or not */
	final List<LazyAABBTreeLeaf2<E, T>> elements;
	
	boolean updateTree = false;
	
	double perimeter, perimeterGood;
	
	/**
	 * Default constructor.
	 */
	public LazyAABBTree2() {
		this(BroadphaseDetector.DEFAULT_INITIAL_CAPACITY);
	}
	
	/**
	 * Optional constructor.
	 * <p>
	 * Allows fine tuning of the initial capacity of local storage for faster running times.
	 * 
	 * @param initialCapacity the initial capacity of local storage
	 * @throws IllegalArgumentException if initialCapacity is less than zero
	 */
	public LazyAABBTree2(int initialCapacity) {
		this.elements = new ArrayList<LazyAABBTreeLeaf2<E, T>>(initialCapacity);
		this.elementMap = new HashMap<BroadphaseKey, LazyAABBTreeLeaf2<E, T>>(initialCapacity);
	}
	
	/**
	 * Destroys the existing tree in O(n) time and prepares for batch-detection while
	 * also updating all AABBs. Called by {@link World} in each step before detection.
	 */
	
	@Override
	public void batchUpdate() {
		for (LazyAABBTreeLeaf2<E, T> leaf : this.elements) {
			leaf.updateAABB();
		}
		
		if (!updateTree()) {
			root.collidingParent = null;
			perimeter = refreshTree(this.root);
		}
	}
	
	boolean updateTree() {
		if (updateTree || perimeter > perimeterGood * 1.1 || root == null) {
			buildTree();
			root.collidingParent = null;
			perimeter = refreshTree(this.root);
			
			perimeterGood = perimeter;
			updateTree = false;
			
			return true;
		}
		
		return false;
	}
	
	double refreshTree(LazyAABBTreeNode2 node) {
		if (node.isLeaf()) {
			return 0.0;
		}
		
		node.left.collidingParent = node.collidingParent;
		node.right.collidingParent = (node.right.aabb.overlaps(node.left.aabb))? node : node.collidingParent;

		double pleft = refreshTree(node.left);
		double pright = refreshTree(node.right);
		
		node.aabb.set(node.left.aabb);
		node.aabb.union(node.right.aabb);
		
		return node.aabb.getWidth() + node.aabb.getHeight() + pleft + pright;
		
		/*
		while (node.left.collidingParent != null && !node.left.aabb.overlaps(node.left.collidingParent.left.aabb)) {
			node.left.collidingParent = node.left.collidingParent.collidingParent;
		}
		
		while (node.right.collidingParent != null && !node.right.aabb.overlaps(node.right.collidingParent.left.aabb)) {
			node.right.collidingParent = node.right.collidingParent.collidingParent;
		}*/
		
		/*
		if (node.left.aabb.overlaps(node.right.aabb)) {
			while (node.collidingParent != null && !node.right.aabb.overlaps(node.collidingParent.left.aabb)) {
				node.collidingParent = node.collidingParent.collidingParent;
			}
			
			rightCParent = node;
		} else {
			rightCParent = cparent;
		}*/
	}
	
	/*
	double refreshTree(LazyAABBTreeNode2 node, LazyAABBTreeNode2 cparent) {
		node.collidingParent = cparent;
		
		if (node.isLeaf()) {
			return 0.0;
		}
		
		LazyAABBTreeNode2 rightCParent = (node.left.aabb.overlaps(node.right.aabb))? node: cparent;

		double pleft = refreshTree(node.left, cparent);
		double pright = refreshTree(node.right, rightCParent);
		
		node.aabb.set(node.left.aabb);
		node.aabb.union(node.right.aabb);
		
		return node.aabb.getPerimeter() + pleft + pright;
	}
	 */
	
	void buildTree() {
		this.root = this.buildTree(null, 0, this.elements.size());
	}
	
	LazyAABBTreeNode2 buildTree(LazyAABBTreeNode2 parent, int start, int end) {
		int count = end - start;
		int group0;
		
		if (count == 1) {
			LazyAABBTreeLeaf2<E, T> leaf = this.elements.get(start);
			//leaf.parent = parent;
			
			return leaf;
		} else if (count == 2) {
			group0 = start + 1;
		} else {
			LazyAABBTreeLeaf2<E, T> n0 = elements.get(start);
			double minx = n0.aabb.getCenterX(), maxx = minx;
			double miny = n0.aabb.getCenterY(), maxy = miny;
			
			for (int i = start + 1; i < end; i++) {
				LazyAABBTreeLeaf2<E, T> node = elements.get(i);
				double cx = node.aabb.getCenterX();
				double cy = node.aabb.getCenterY();
				
				if (cx < minx) {
					minx = cx;
				} else if (cx > maxx) {
					maxx = cx;
				}
				
				if (cy < miny) {
					miny = cy;
				} else if (cy > maxy) {
					maxy = cy;
				}
			}
			
			boolean splitX = (maxx - minx) > (maxy - miny);
			double mid = splitX? (minx + maxx) / 2 : (miny + maxy) / 2;
			group0 = start;
			
			for (int i = start; i < end; i++) {
				LazyAABBTreeLeaf2<E, T> node = elements.get(i);
				double nodeMid = splitX? node.aabb.getCenterX() : node.aabb.getCenterY();
				
				if (nodeMid < mid) {
					elements.set(i, elements.get(group0));
					elements.set(group0, node);
					
					group0++;
				}
			}
			
			if (group0 == start || group0 == end) {
				group0 = (start + end) / 2;
			}
		}
		
		LazyAABBTreeNode2 cur = new LazyAABBTreeNode2();
		//cur.parent = parent;
		cur.left = buildTree(cur, start, group0);
		cur.right = buildTree(cur, group0, end);
		cur.aabb = new AABB(0, 0, 0, 0);//cur.left.aabb.getUnion(cur.right.aabb);
		
		return cur;
	}
	
	/* (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#add(org.dyn4j.collision.Collidable, org.dyn4j.collision.Fixture)
	 */
	@Override
	public void add(E collidable, T fixture) {
		// create a new node for the collidable
		BroadphaseKey key = BroadphaseKey.get(collidable, fixture);
		LazyAABBTreeLeaf2<E, T> existing = this.elementMap.get(key);
		
		if (existing != null) {
			existing.updateAABB();
			
			/*LazyAABBTreeNode2 node = existing.parent;
			while (node != null) {
				double prev = node.aabb.getPerimeter();
				node.aabb.union(existing.aabb);
				perimeter = perimeter - prev + node.aabb.getPerimeter();
				node = node.parent;
			}*/
		} else {
			// add new node
			LazyAABBTreeLeaf2<E, T> node = new LazyAABBTreeLeaf2<E, T>(collidable, fixture);
			
			this.elementMap.put(key, node);
			this.elements.add(node);
			
			this.updateTree = true;
		}
	}
	
	/* (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#remove(org.dyn4j.collision.Collidable, org.dyn4j.collision.Fixture)
	 */
	@Override
	public boolean remove(E collidable, T fixture) {
		BroadphaseKey key = BroadphaseKey.get(collidable, fixture);
		// find the node in the map
		LazyAABBTreeLeaf2<E, T> node = this.elementMap.remove(key);
		// make sure it was found
		
		if (node != null) {
			this.elements.remove(node);
			this.updateTree = true;
			return true;
		}
		
		return false;
	}
	
	/* (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#update(org.dyn4j.collision.Collidable, org.dyn4j.collision.Fixture)
	 */
	@Override
	public void update(E collidable, T fixture) {
		// In the way the add and update are described in BroadphaseDetector, their functionallity is identical
		// so just redirect the work to add for less duplication.
		this.add(collidable, fixture);
	}
	
	/* (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#getAABB(org.dyn4j.collision.Collidable, org.dyn4j.collision.Fixture)
	 */
	@Override
	public AABB getAABB(E collidable, T fixture) {
		BroadphaseKey key = BroadphaseKey.get(collidable, fixture);
		LazyAABBTreeLeaf2<E, T> node = this.elementMap.get(key);
		
		if (node != null) {
			return node.aabb;
		}
		
		return fixture.getShape().createAABB(collidable.getTransform());
	}
	
	/* (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#contains(org.dyn4j.collision.Collidable, org.dyn4j.collision.Fixture)
	 */
	@Override
	public boolean contains(E collidable, T fixture) {
		BroadphaseKey key = BroadphaseKey.get(collidable, fixture);
		return this.elementMap.containsKey(key);
	}
	
	/* (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#clear()
	 */
	@Override
	public void clear() {
		this.elementMap.clear();
		this.elements.clear();
		this.root = null;
		
		// Important: since everything is removed there's no pending work to do
		this.updateTree = false;
	}
	
	/* (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#size()
	 */
	@Override
	public int size() {
		return this.elements.size();
	}
	
	/* (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#detect(org.dyn4j.collision.broadphase.BroadphaseFilter)
	 */
	@Override
	public List<BroadphasePair<E, T>> detect(BroadphaseFilter<E, T> filter) {
		int size = this.elements.size();
		int eSize = Collisions.getEstimatedCollisionPairs(size);
		List<BroadphasePair<E, T>> pairs = new ArrayList<BroadphasePair<E, T>>(eSize);
		
		this.updateTree();
		
		for (int i = 0; i < size; i++) {
			LazyAABBTreeLeaf2<E, T> leaf = this.elements.get(i);
			LazyAABBTreeNode2 node = leaf.collidingParent;
			
			while (node != null) {
				if (node.left.aabb.overlaps(leaf.aabb)) {
					this.detect(leaf, node.left, filter, pairs);	
				}
				
				node = node.collidingParent;
			}
		}
		
		return pairs;
	}
	
	/**
	 * Internal recursive method to detect broad-phase collisions while building the tree. Only used from insertAndDetect.
	 * Caution: Assumes that node collides with root.aabb when called (In order to reduce recursion height).
	 * Note that in contrast to {@link DynamicAABBTree} we don't need to check if one node was tested for collision.
	 * Because the nodes are tested while being inserted each pair will only be tested once, so we skip those tests.
	 */
	private void detect(LazyAABBTreeLeaf2<E, T> node, LazyAABBTreeNode2 root, BroadphaseFilter<E, T> filter, List<BroadphasePair<E, T>> pairs) {
		// test the node itself
		// check for leaf node
		// non-leaf nodes always have a left child
		if (root.isLeaf()) {
			@SuppressWarnings("unchecked")
			LazyAABBTreeLeaf2<E, T> leaf = (LazyAABBTreeLeaf2<E, T>) root;
			
			if (filter.isAllowed(node.collidable, node.fixture, leaf.collidable, leaf.fixture)) {
				BroadphasePair<E, T> pair = new BroadphasePair<E, T>(
						node.collidable,	// A
						node.fixture,
						leaf.collidable,	// B
						leaf.fixture);	
				// add the pair to the list of pairs
				pairs.add(pair);
			}
		} else {
			// they overlap so descend into both children
			if (node.aabb.overlaps(root.left.aabb)) this.detect(node, root.left, filter, pairs);
			if (node.aabb.overlaps(root.right.aabb)) this.detect(node, root.right, filter, pairs);
		}
	}
	
	/* (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#detect(org.dyn4j.geometry.AABB)
	 */
	@Override
	public List<BroadphaseItem<E, T>> detect(AABB aabb, BroadphaseFilter<E, T> filter) {
		this.updateTree();
		
		if (this.root == null) {
			return Collections.emptyList();
		}
		
		int eSize = Collisions.getEstimatedCollisionsPerObject();
		List<BroadphaseItem<E, T>> list = new ArrayList<BroadphaseItem<E, T>>(eSize);
		
		if (aabb.overlaps(this.root.aabb)) {
			this.detect(aabb, this.root, filter, list);	
		}
		
		return list;
	}
	
	/**
	 * Internal recursive method used to implement BroadphaseDetector#detect.
	 * @param aabb the aabb to test with
	 * @param node the node to begin at
	 * @param filter the filter
	 * @param list the results list
	 */
	private void detect(AABB aabb, LazyAABBTreeNode2 node, BroadphaseFilter<E, T> filter, List<BroadphaseItem<E, T>> list) {
		// test the node itself
		// check for leaf node
		// non-leaf nodes always have a left child
		
		if (node.isLeaf()) {
			@SuppressWarnings("unchecked")
			LazyAABBTreeLeaf2<E, T> leaf = (LazyAABBTreeLeaf2<E, T>)node;
			// its a leaf so add the collidable
			if (filter.isAllowed(aabb, leaf.collidable, leaf.fixture)) {
				list.add(new BroadphaseItem<E, T>(leaf.collidable, leaf.fixture));
			}
			// return and check other limbs
		} else {
			// they overlap so descend into both children
			if (aabb.overlaps(node.left.aabb)) this.detect(aabb, node.left, filter, list);
			if (aabb.overlaps(node.right.aabb)) this.detect(aabb, node.right, filter, list);
		}
	}
	
	/* (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#raycast(org.dyn4j.geometry.Ray, double)
	 */
	@Override
	public List<BroadphaseItem<E, T>> raycast(Ray ray, double length, BroadphaseFilter<E, T> filter) {
		this.updateTree();
		
		if (this.root == null) {
			return Collections.emptyList();
		}
		
		// create an aabb from the ray
		Vector2 s = ray.getStart();
		Vector2 d = ray.getDirectionVector();
		
		// get the length
		if (length <= 0.0) {
			length = Double.MAX_VALUE;
		}
		
		// create the aabb
		double w = d.x * length;
		double h = d.y * length;
		AABB aabb = AABB.createAABBFromPoints(s.x, s.y, s.x + w, s.y + h);
		
		if (!root.aabb.overlaps(aabb)) {
			return Collections.emptyList();
		}
		
		// precompute
		double invDx = 1.0 / d.x;
		double invDy = 1.0 / d.y;
		
		// get the estimated collision count
		int eSize = Collisions.getEstimatedRaycastCollisions(this.elementMap.size());
		List<BroadphaseItem<E, T>> items = new ArrayList<BroadphaseItem<E, T>>(eSize);
		LazyAABBTreeNode2 node = this.root;
		
		/*
		// perform a iterative, stack-less, traversal of the tree
		while (node != null) {
			// check if the current node overlaps the desired node
			if (aabb.overlaps(node.aabb)) {
				// if they do overlap, then check the left child node
				if (node.isLeaf()) {
					if (this.raycast(s, length, invDx, invDy, node.aabb)) {
						// if both are null, then this is a leaf node
						@SuppressWarnings("unchecked")
						LazyAABBTreeLeaf2<E, T> leaf = (LazyAABBTreeLeaf2<E, T>)node;
						if (filter.isAllowed(ray, length, leaf.collidable, leaf.fixture)) {
							items.add(new BroadphaseItem<E, T>(leaf.collidable, leaf.fixture));
						}
						// if its a leaf node then we need to go back up the
						// tree and test nodes we haven't yet
					}
				} else {
					// if the left is not null, then check that subtree
					node = node.left;
					continue;
				}
			}
			
			// if the current node is a leaf node or doesnt overlap the
			// desired aabb, then we need to go back up the tree until we
			// find the first left node who's right node is not null
			boolean nextNodeFound = false;
			while (node.parent != null) {
				// check if the current node the left child of its parent
				if (node == node.parent.left) {
					// it is, so check if the right node is non-null
					// NOTE: not need since the tree is a complete tree (every node has two children)
					//if (node.parent.right != null) {
					// it isn't so the sibling node is the next node
					node = node.parent.right;
					nextNodeFound = true;
					break;
					//}
				}
				
				// if the current node isn't a left node or it is but its
				// sibling is null, go to the parent node
				node = node.parent;
			}
			
			// if we didn't find it then we are done
			if (!nextNodeFound) break;
		}
		*/
		return items;
	}
	
	/* (non-Javadoc)
	 * @see org.dyn4j.geometry.Shiftable#shift(org.dyn4j.geometry.Vector2)
	 */
	@Override
	public void shift(Vector2 shift) {
		// make sure the tree is built
		this.updateTree();
		
		// Left intact from DynamicAABBTree
		
		// we need to update all nodes in the tree (not just the
		// nodes that contain the bodies)
		LazyAABBTreeNode2 node = this.root;
		// perform a iterative, stack-less, in order traversal of the tree
		/*
		while (node != null) {
			// traverse down the left most tree first
			if (node.left != null) {
				node = node.left;
			} else if (node.right != null) {
				// if the left sub tree is null then go
				// down the right sub tree
				node.aabb.translate(shift);
				node = node.right;
			} else {
				// if both sub trees are null then go back
				// up the tree until we find the first left
				// node who's right node is not null
				node.aabb.translate(shift);
				boolean nextNodeFound = false;
				while (node.parent != null) {
					if (node == node.parent.left) {
						if (node.parent.right != null) {
							node.parent.aabb.translate(shift);
							node = node.parent.right;
							nextNodeFound = true;
							break;
						}
					}
					node = node.parent;
				}
				if (!nextNodeFound) break;
			}
		}*/
	}
	
	/*
	 * Ideally setAABBExpansion would throw an unsupported operation exception because we don't want to expand the AABBs in any case.
	 * But this could break existing applications that explicitly set the expansion in the case that this broadphase is set as the default.
	 * So we'll be transparent and just ignore the value.
	 * 
	 * @Override
	 * public void setAABBExpansion(double expansion) {
	 * 	throw new UnsupportedOperationException();
	 * }
	 */
	
	/*
	 * But at least if the user asks, let them know that expansion is logically 0.
	 * 
	 * (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#getAABBExpansion()
	 */
	@Override
	public double getAABBExpansion() {
		return 0;
	}
	
	/* 
	 * (non-Javadoc)
	 * @see org.dyn4j.collision.broadphase.BroadphaseDetector#supportsAABBExpansion()
	 */
	@Override
	public boolean supportsAABBExpansion() {
		return false;
	}
	
}