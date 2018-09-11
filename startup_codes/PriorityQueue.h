/*
 * PriorityQueue.h
 *
 *  Created on: 2018��9��7��
 *      Author: ygli
 */

#ifndef PRIORITYQUEUE_H_
#define PRIORITYQUEUE_H_

#include <queue>
#include "globalVariables.h"

class PriorityQueue {
public:
	PriorityQueue() {
	}

	virtual ~PriorityQueue() {

	}

	int size() {
		return m_items.size();
	}

	bool empty() {
		return m_items.empty();
	}

	void insert(MazeCell* e) {
		m_items.push_back(e);
		std::push_heap(m_items.begin(), m_items.end(), comparator);
	}

	MazeCell* find(int row, int col) {
		for (int i = 0; i < size(); i++) {
			if (m_items[i]->equals(row, col))
				return m_items[i];
		}

		return nullptr;
	}

	/* do nothing if [row, col] is not in the queue */
	void remove(int row, int col) {
		for (int i = 0; i < size(); i++) {
			if (!m_items[i]->equals(row, col))
				continue;

			m_items.erase(m_items.begin() + i);
			std::make_heap(m_items.begin(), m_items.end(), comparator);
			break;
		}
	}

	MazeCell* pop() {
		if (size() == 0)
			return nullptr;
		MazeCell* t = m_items.front();
		std::pop_heap(m_items.begin(), m_items.end(), comparator);
		m_items.pop_back();
		return t;
	}

	MazeCell* top() {
		if (size() == 0)
			return nullptr;

		return m_items.front();
	}

	static bool comparator(MazeCell* c1, MazeCell* c2) {
		if (c1->key[0] == c2->key[0])
			return c1->key[1] > c2->key[1];
		return c1->key[0] > c2->key[0];
	}

private:
	vector<MazeCell*> m_items;
};

#endif /* PRIORITYQUEUE_H_ */