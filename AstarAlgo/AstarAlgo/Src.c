#include <stdio.h>
#include <Windows.h>
#include <stdlib.h>

typedef int TileBlk[4][4];
typedef struct anode *Ty_nodeptr;
typedef struct anode {
	TileBlk tile;
	double f_hat;
	double g_hat;
	double h_hat;
	Ty_nodeptr pred;
	int dir; // 방향을 위한 변수 값 추가 (0: 위, 1: 아래, 2: 좌, 3: 우)
}Ty_node;

typedef struct linknode * Ty_linknodeptr;
typedef struct linknode {
	Ty_nodeptr nodeptr;
	Ty_linknodeptr next;
}Ty_linknode;

Ty_linknodeptr open=NULL;
Ty_linknodeptr closed=NULL;
Ty_linknodeptr result_node= NULL; //가장 마지막 노드를 저장한다.
Ty_nodeptr cur_n;	// 현재 노드 n
Ty_nodeptr goal;	// goal 노드

void init();	// start 노드와 goal 노드를 입력받고 초기화 시켜준다.
void set_cur_n();	// 현재 노드 n을 설정해준다.
void cal_suc(Ty_nodeptr n);	// 노드 n의 successor들을 구한다.
void cal_f_hat(Ty_nodeptr node);	// Ty_node의 f,g,h hat을 구한다.
void cal_h_hat(Ty_nodeptr node);
int cmp_match(Ty_nodeptr node1,Ty_nodeptr node2);	//node1과 node2를 비교해서 0(다르다) 또는 1(같다)를 출력한다.
int is_in_open(Ty_nodeptr node);					//node가 open에 있는지를 확인한다. 0(없음.) 1(있음.)
int is_in_closed(Ty_nodeptr node);					//node가 closed에 있는지를 확인한다. 0(없음.) 1(있음.)
void insert_into_open(Ty_nodeptr node);				//node를 open에 f_hat이 작은 순으로 넣는다.
void insert_into_closed(Ty_nodeptr node);			//node를 closed에 넣는다.
void closed_to_open(Ty_nodeptr node);				//closed에 있는 node를 open으로 옮겨 준다.
int compare_f_in_open(Ty_nodeptr node);				//node의 new f와 open의 old f를 비교해 new f가 old보다 작으면 1을 반환한다.
int compare_f_in_closed(Ty_nodeptr node);			//node의 new f와 closed의 old f를 비교해 new f가 old보다 작으면 1을 반환한다.
void update_f_g(Ty_nodeptr node);					//open에 있는 node의 f_hat과 g_hat값을 업데이트 시켜준다.
void update_pred(Ty_nodeptr node, Ty_nodeptr n);	//open에 있는 node의 pred값을 업데이트 시켜준다.
void copy_tile(Ty_nodeptr origin, Ty_nodeptr result, int kind); //result의 값을 origin으로 복사한다. 단, kind의 값에 따라 -1의 값을 상(0),하(1),좌(2),우(3)으로 이동시킨다.
int goal_check(Ty_nodeptr node);					//node가 goal_node이면 1을 반환하고 아닐경우 0을 반환한다.
void print_result();								//closed에 있는 노드들을 참조하여 경로를 출력한다.



int main() {
	// 처음 받은 start node를 현재 노드 n으로 설정까지 해준다.
	init();	
	// success(n)을 구하고 open에 넣어주는 과정. 
	//여기까지는 open에 start node의 successor들이 들어가 있고, closed에 start node이 들어가 있다.
	do {
		set_cur_n();
		cal_suc(cur_n);
	} while (!goal_check(cur_n));
	print_result();
	return 0;
}

void init() {
	FILE *fp;
	Ty_nodeptr start_init=(Ty_nodeptr) malloc(sizeof(Ty_node));
	Ty_nodeptr goal_init = (Ty_nodeptr)malloc(sizeof(Ty_node));
	int i, j;
	printf("Read start node from file...\n");
	fp = fopen("startnode.txt", "r");
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 3; j++) {
			fscanf(fp,"%d\t", &start_init->tile[i][j]);
		}
		fscanf(fp,"%d", &start_init->tile[i][j]);
	}
	printf("\nRead goal node from file...\n");
	fclose(fp);
	fp = fopen("goalnode.txt", "r");
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 3; j++) {
			fscanf(fp,"%d\t", &goal_init->tile[i][j]);
		}
		fscanf(fp,"%d", &goal_init->tile[i][j]);
	}
	printf("\nInitiate node complete.\n");
	fclose(fp);
	//처음 받은 start node를 현재 노드 n으로 설정까지 해준다. goal_init node는 goal node로 설정해 준다.
	cur_n = start_init;
	goal = goal_init;
	//start node의 hat 변수들은 이 함수 내에서 자체적으로 계산해 준다.
	start_init->g_hat = 0.0;
	cal_h_hat(start_init);
	start_init->f_hat = start_init->g_hat + start_init->h_hat;
	start_init->pred = NULL;
	closed = (Ty_linknodeptr)malloc(sizeof(Ty_linknode));
	closed->next = NULL;
	closed->nodeptr = NULL;
	open = (Ty_linknodeptr)malloc(sizeof(Ty_linknode));
	open->next = NULL;
	open->nodeptr = NULL;
	//closed->nodeptr = start_init;	//start_init은 open에 넣었다가 바로 closed에 넣어졌다고 생각한다.
	insert_into_open(start_init);
	system("cls");
}

// 현재 노드 n을 설정해준다.
// 먼저 현재 노드 n을 closed에 넣고 open의 제일 첫 번째 노드를 참조하여 새로운 현재 노드를 생성한다.
void set_cur_n()
{
	Ty_linknodeptr tmp=NULL;
	int i, j;
	cur_n = open->nodeptr;
	if (cur_n->pred != NULL && !cmp_match(cur_n->pred,closed->nodeptr)) {
		do {
			if (cmp_match(result_node->nodeptr, cur_n->pred))
				break;
			else {
				result_node = result_node->next;
			}
		} while (result_node != NULL);
	}
	insert_into_closed(cur_n);
	cur_n = (Ty_nodeptr)malloc(sizeof(Ty_node));
	cur_n->f_hat = open->nodeptr->f_hat;
	if (open->nodeptr->pred != NULL) {
		cur_n->g_hat = open->nodeptr->pred->g_hat + 1;
	}
	else {
		cur_n->g_hat = open->nodeptr->g_hat;
	}
	cur_n->h_hat = open->nodeptr->h_hat;
	cur_n->dir = open->nodeptr->dir;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			cur_n->tile[i][j] = open->nodeptr->tile[i][j];
		}
	}
	cur_n->pred = open->nodeptr->pred;
	open = open->next;
	if (result_node == NULL) {
		result_node= (Ty_linknodeptr)malloc(sizeof(Ty_linknode));
		result_node->nodeptr = cur_n;
		result_node->next = NULL;
	}
	else {
		tmp = (Ty_linknodeptr)malloc(sizeof(Ty_linknode));
		tmp->nodeptr = cur_n;
		tmp->next = result_node;
		result_node = tmp;
	}
}


//-1이 움직일 수 있는 4가지(경우에 따라 2,3가지가 될 수 있다.) 타일의 정보를 구한다.
void cal_suc(Ty_nodeptr n) {
	
	int i, j;
	Ty_nodeptr left = NULL;
	Ty_nodeptr right = NULL;
	Ty_nodeptr up = NULL;
	Ty_nodeptr down = NULL;
	int in_open=0;
	int in_closed=0;

	//-1의 위치에 따라 left,up,down,right 노드를 각각 생성해 준다.
	//만약 생성이 되지 않는 노드가 있다면 NULL로 유지될 것이다.
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			if (n->tile[i][j] == -1) {
				if (i == 0) {
					if (j == 0) {
						//left와 up이 생성이 되지 않는다.
						right = (Ty_nodeptr)malloc(sizeof(Ty_node));
						down = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, right, 3);
						copy_tile(n, down, 1);
					}
					else if (j == 3) {
						//up과 right이 생성이 되지 않는다.
						left = (Ty_nodeptr)malloc(sizeof(Ty_node));
						down = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, left, 2);
						copy_tile(n, down, 1);
					}
					else {
						//up이 생성이 되지 않는다.
						left = (Ty_nodeptr)malloc(sizeof(Ty_node));
						down = (Ty_nodeptr)malloc(sizeof(Ty_node));
						right = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, right, 3);
						copy_tile(n, left, 2);
						copy_tile(n, down, 1);
					}
				}
				else if (j == 0) {
					if (i == 3) {
						//left와 down이 생성이 되지 않는다.
						right = (Ty_nodeptr)malloc(sizeof(Ty_node));
						up = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, right, 3);
						copy_tile(n, up, 0);
					}
					else {
						//left가 생성이 되지 않는다.
						down = (Ty_nodeptr)malloc(sizeof(Ty_node));
						right = (Ty_nodeptr)malloc(sizeof(Ty_node));
						up = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, right, 3);
						copy_tile(n, down, 1);
						copy_tile(n, up, 0);
					}
				}
				else if (i == 3) {
					if (j == 3) {
						//right와 down이 생성이 되지 않는다.
						left = (Ty_nodeptr)malloc(sizeof(Ty_node));
						up = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, left, 2);
						copy_tile(n, up, 0);
					}
					else {
						//down이 생성이 되지 않는다.
						left = (Ty_nodeptr)malloc(sizeof(Ty_node));
						right = (Ty_nodeptr)malloc(sizeof(Ty_node));
						up = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, right, 3);
						copy_tile(n, left, 2);
						copy_tile(n, up, 0);
					}
				}
				else if (j == 3) {
					//right이 생성이 되지 않는다.
					left = (Ty_nodeptr)malloc(sizeof(Ty_node));
					down = (Ty_nodeptr)malloc(sizeof(Ty_node));
					up = (Ty_nodeptr)malloc(sizeof(Ty_node));
					copy_tile(n, left, 2);
					copy_tile(n, up, 0);
					copy_tile(n, down, 1);
				}
				else {
					//모두 생성이 된다.
					right = (Ty_nodeptr)malloc(sizeof(Ty_node));
					left = (Ty_nodeptr)malloc(sizeof(Ty_node));
					down = (Ty_nodeptr)malloc(sizeof(Ty_node));
					up = (Ty_nodeptr)malloc(sizeof(Ty_node));
					copy_tile(n, right, 3);
					copy_tile(n, left, 2);
					copy_tile(n, up, 0);
					copy_tile(n, down, 1);
				}
			}
		}
	}

	//각각의 success들이 open과 closed에 있는지 없는지에 따라 적절한 행동을 취한다.
	if (left != NULL) {
		in_open = is_in_open(left);
		in_closed = is_in_closed(left);
		//open과 closed에 둘 다 없으면 open에 넣고 pred를 n으로 지정해 준다.
		left->dir = 2;
		if (in_open == 0 && in_closed == 0) {
			insert_into_open(left);
			left->pred = n;
		}
		else {
			if (in_closed == 1) {
				if (compare_f_in_closed(left)) {
					closed_to_open(left);
					update_f_g(left);
					update_pred(left, n);
				}
			}else if (in_open == 1) {
				if (compare_f_in_open(left)) {
					update_f_g(left);
					update_pred(left, n);
				}
				else {

				}
			}
		}
	}
	if (right != NULL) {
		in_open = is_in_open(right);
		in_closed = is_in_closed(right);
		right->dir = 3;
		//open과 closed에 둘 다 없으면 open에 넣고 pred를 n으로 지정해 준다.
		if (in_open == 0 && in_closed == 0) {
			insert_into_open(right);
			right->pred = n;
		}
		else {
			if (in_closed == 1) {
				if (compare_f_in_closed(right)) {
					closed_to_open(right);
					update_f_g(right);
					update_pred(right, n);
				}
			}
			else if (in_open == 1) {
				if (compare_f_in_open(right)) {
					update_f_g(right);
					update_pred(right, n);
				}
			}
		}
	}
	if (up != NULL) {
		in_open = is_in_open(up);
		in_closed = is_in_closed(up);
		up->dir = 0;
		//open과 closed에 둘 다 없으면 open에 넣고 pred를 n으로 지정해 준다.
		if (in_open == 0 && in_closed == 0) {
			insert_into_open(up);
			up->pred = n;
		}
		else {
			if (in_closed == 1) {
				if (compare_f_in_closed(up)) {
					closed_to_open(up);
					update_f_g(up);
					update_pred(up, n);
				}
			}
			else if (in_open == 1) {
				if (compare_f_in_open(up)) {
					update_f_g(up);
					update_pred(up, n);
				}
			}
		}
	}
	if (down != NULL) {
		in_open = is_in_open(down);
		in_closed = is_in_closed(down);
		down->dir = 1;
		//open과 closed에 둘 다 없으면 open에 넣고 pred를 n으로 지정해 준다.
		if (in_open == 0 && in_closed == 0) {
			insert_into_open(down);
			down->pred = n;
		}
		else {
			if (in_closed == 1) {
				if (compare_f_in_closed(down)) {
					closed_to_open(down);
					update_f_g(down);
					update_pred(down, n);
				}
			}
			else if (in_open == 1) {
				if (compare_f_in_open(down)) {
					update_f_g(down);
					update_pred(down, n);
				}
			}
		}
	}
}

//origin의 값을 result로 복사한다. 단, kind의 값에 따라 -1의 값을 상(0),하(1),좌(2),우(3)으로 이동시킨다.
//result의 hat값들도 계산해 준다.
void copy_tile(Ty_nodeptr origin, Ty_nodeptr result, int kind) {
	// tile에서 -1인 값을 갖는 위치 정보
	int minus_i, minus_j;
	int i, j;
	// tile에서 -1인 값을 갖는 위치 정보를 찾는다.
	for (minus_i = 0; minus_i < 4; minus_i++) {
		for (minus_j = 0; minus_j < 4; minus_j++) {
			if (origin->tile[minus_i][minus_j] == -1) {
				break;
			}
		}
		if (origin->tile[minus_i][minus_j] == -1) {
			break;
		}
	}
	switch (kind) {
	case 0://상
		//-1 타일 값과 -1 타일의 위의 값을 제외한 모든 origin 타일의 정보를 result로 복사한다.
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++) {
				if (!(i == minus_i&&j == minus_j) && !(i == minus_i - 1 && j == minus_j)) {
					result->tile[i][j] = origin->tile[i][j];
				}
			}
		}
		result->tile[minus_i - 1][minus_j] = origin->tile[minus_i][minus_j];
		result->tile[minus_i][minus_j] = origin->tile[minus_i - 1][minus_j];
		result->g_hat = origin->g_hat+1.0;
		cal_h_hat(result);
		cal_f_hat(result);
		break;
	case 1://하
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++) {
				if (!(i == minus_i&&j == minus_j) && !(i == minus_i + 1 && j == minus_j)) {
					result->tile[i][j] = origin->tile[i][j];
				}
			}
		}
		result->tile[minus_i + 1][minus_j] = origin->tile[minus_i][minus_j];
		result->tile[minus_i][minus_j] = origin->tile[minus_i + 1][minus_j];
		result->g_hat = origin->g_hat + 1.0;
		cal_h_hat(result);
		cal_f_hat(result);
		break;
	case 2://좌
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++) {
				if (!(i == minus_i&&j == minus_j) && !(i == minus_i && j == minus_j - 1)) {
					result->tile[i][j] = origin->tile[i][j];
				}
			}
		}
		result->tile[minus_i][minus_j - 1] = origin->tile[minus_i][minus_j];
		result->tile[minus_i][minus_j] = origin->tile[minus_i][minus_j - 1];
		result->g_hat = origin->g_hat + 1.0;
		cal_h_hat(result);
		cal_f_hat(result);
		break;
	case 3://우
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++) {
				if (!(i == minus_i&&j == minus_j) && !(i == minus_i && j == minus_j + 1)) {
					result->tile[i][j] = origin->tile[i][j];
				}
			}
		}
		result->tile[minus_i][minus_j + 1] = origin->tile[minus_i][minus_j];
		result->tile[minus_i][minus_j] = origin->tile[minus_i][minus_j + 1];
		result->g_hat = origin->g_hat + 1.0;
		cal_h_hat(result);
		cal_f_hat(result);
		break;
	}
}


// start node를 제외한 나머지 node들의 f hat, g hat, h hat을 구하는 함수들이다. 
// start node는 f hat을 구하는 함수를 사용하지 않는다.
void cal_f_hat(Ty_nodeptr node) {
	node->f_hat = (node->g_hat) + (node->h_hat);
}
// goal node와 parameter node의 타일 차이 수를 h_hat에 대입해 준다.
void cal_h_hat(Ty_nodeptr node) {
	int i, j;
	int diff_cnt=0;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			if (node->tile[i][j] != goal->tile[i][j]&&node->tile[i][j]!=-1&& goal->tile[i][j] != -1) {
				diff_cnt++;
			}
		}
	}
	node->h_hat = (double)diff_cnt;
}
//node1과 node2를 비교해서 0(다르다) 또는 1(같다)를 출력한다.
int cmp_match(Ty_nodeptr node1, Ty_nodeptr node2)	
{
	int i, j;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			if (node1->tile[i][j] != node2->tile[i][j]) {
				return 0;
			}
		}
	}
	return 1;
}
//node가 open에 있는지를 확인한다. 0(없음.) 1(있음.)
int is_in_open(Ty_nodeptr node)					
{
	Ty_linknodeptr tmp = open;
	if (open == NULL) {
		return 0;
	}
	else {
		do {
			if (tmp->nodeptr != NULL) {
				if (cmp_match(node, tmp->nodeptr)) {
					return 1;
				}
			}
			tmp = tmp->next;
		} while (tmp != NULL);
	}
	return 0;
}
//node가 closed에 있는지를 확인한다. 0(없음.) 1(있음.)
int is_in_closed(Ty_nodeptr node)					
{
	Ty_linknodeptr tmp = closed;
	if (open == NULL) {
		return 0;
	}
	else {
		do {
			if (cmp_match(node, tmp->nodeptr)) {
				return 1;
			}
			tmp = tmp->next;
		} while (tmp->nodeptr != NULL);
	}
	return 0;
}
//node를 open에 f_hat이 작은 순으로 넣는다.
void insert_into_open(Ty_nodeptr node)				
{
	Ty_linknodeptr tmp = open;
	Ty_linknodeptr tmp2 = NULL;
	if (open== NULL) {
		tmp2 = (Ty_linknodeptr)malloc(sizeof(Ty_linknode));
		tmp2->nodeptr = node;
		tmp2->next = NULL;
		open = tmp2;
		return ;
	}
	if (open->nodeptr == NULL) {
		open->nodeptr = node;
		return ;
	}
	if (open->nodeptr->f_hat >= node->f_hat) {
		tmp2 = (Ty_linknodeptr)malloc(sizeof(Ty_linknode));
		tmp2->nodeptr = node;
		tmp2->next = open;
		open = tmp2;
		return ;
	}
	do {
		if (tmp->next != NULL) {
			if (tmp->next->nodeptr->f_hat >= node->f_hat) {
				tmp2 = (Ty_linknodeptr)malloc(sizeof(Ty_linknode));
				tmp2->nodeptr = node;
				tmp2->next = tmp->next;
				tmp->next = tmp2;
				return ;
			}
		}
		else {
			tmp2 = (Ty_linknodeptr)malloc(sizeof(Ty_linknode));
			tmp2->nodeptr = node;
			tmp2->next = NULL;
			tmp->next = tmp2;
			return ;
		}
		tmp = tmp->next;
	} while (tmp != NULL);
}
//node를 closed에 넣는다.
void insert_into_closed(Ty_nodeptr node)			
{
	Ty_linknodeptr tmp = (Ty_linknodeptr)malloc(sizeof(Ty_linknode));
	tmp->next = closed;
	tmp->nodeptr = node;
	closed = tmp;
}
//closed에 있는 node를 open으로 옮겨 준다.
void closed_to_open(Ty_nodeptr node)				
{
	Ty_linknodeptr tmp = closed;
	Ty_linknodeptr tmp2 = NULL;
	Ty_linknodeptr tmp_open = open;
	if (cmp_match(closed->nodeptr, node)) {
		closed = closed->next;
		insert_into_open(tmp->nodeptr);
		return 0;
	}
	tmp = closed;
	do {
		if (cmp_match(tmp->nodeptr, node)) {
			break;
		}
		tmp2 = tmp;
		tmp = tmp->next;
	} while (tmp != NULL);
	insert_into_open(tmp->nodeptr);
	tmp2->next = tmp->next;
	return 0;
}
//node의 new f와 open의 old f를 비교해 new f가 old보다 작으면 1을 반환한다.
int compare_f_in_open(Ty_nodeptr node)				
{
	Ty_linknodeptr tmp = open;
	do {
		if (cmp_match(tmp->nodeptr, node)) {
			if (tmp->nodeptr->f_hat >= node->f_hat) {
				return 1;
			}
		}
		tmp = tmp->next;
	} while (tmp != NULL);
	return 0;
}
//node의 new f와 closed의 old f를 비교해 new f가 old보다 작으면 1을 반환한다.
int compare_f_in_closed(Ty_nodeptr node)			
{
	Ty_linknodeptr tmp = closed;
	do {
		if (cmp_match(tmp->nodeptr, node)) {
			if (tmp->nodeptr->f_hat >= node->f_hat) {
				return 1;
			}
		}
		tmp = tmp->next;
	} while (tmp->nodeptr != NULL);
	return 0;
}
//open에 있는 node의 f_hat과 g_hat값을 업데이트 시켜준다.
void update_f_g(Ty_nodeptr node)					
{
	Ty_linknodeptr tmp = open;
	do {
		if (cmp_match(tmp->nodeptr, node)) {
			tmp->nodeptr->f_hat = node->f_hat;
			tmp->nodeptr->g_hat = node->g_hat;
		}
		tmp = tmp->next;
	} while (tmp != NULL);
}
//open에 있는 node의 pred값을 업데이트 시켜준다.
void update_pred(Ty_nodeptr node, Ty_nodeptr n)
{
	Ty_linknodeptr tmp = open;
	do {
		if (cmp_match(tmp->nodeptr, node)) {
			tmp->nodeptr->pred = n;
			break;
		}
		tmp = tmp->next;
	} while (tmp != NULL);
}
//node가 goal_node이면 1을 반환하고 아닐경우 0을 반환한다.
int goal_check(Ty_nodeptr node) 
{
	if (cmp_match(goal, node)) {
		return 1;
	}
	return 0;
}
//closed에 있는 노드들을 참조하여 경로를 출력한다.
void print_result() 
{
	Ty_nodeptr tmp = result_node->nodeptr;
	Ty_linknodeptr tmp2 = NULL;
	Ty_linknodeptr tmp3 = NULL;
	int cnt=0;
	int i,j;
	printf("The path from the start node to the end node is... \n");
	
	while (tmp != NULL) {
		tmp2 = tmp3;
		tmp3 = (Ty_linknodeptr)malloc(sizeof(Ty_linknode));
		tmp3->nodeptr = tmp;
		tmp3->next = tmp2;
		tmp = tmp->pred;
	}
	tmp2 = tmp3;
	while (1) {
		printf("%d time move\t", cnt);
		switch (tmp3->nodeptr->dir) {
		case 0:
			printf("up\n");
			break;
		case 1:
			printf("down\n");
			break;
		case 2:
			printf("left\n");
			break;
		case 3:
			printf("right\n");
			break;
		default:
			printf("\n");
			break;
		}
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 3; j++) {
				if (tmp3->nodeptr->tile[i][j] == -1) {
					printf(" \t");
				}
				else {
					printf("%d\t", tmp3->nodeptr->tile[i][j]);
				}
			}
			if (tmp3->nodeptr->tile[i][j] == -1) {
				printf(" \t\n");
			}
			else {
				printf("%d\n", tmp3->nodeptr->tile[i][j]);
			}
		}
		printf("ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ\n");
		if (tmp3->next== NULL) {
			break;
		}
		tmp3 = tmp3->next;
		cnt++;
	}
	printf("Goal! ( %d moves )\n",cnt);
	free(open);
	free(closed);
	free(result_node);
	free(tmp3);
	free(tmp2);
}
