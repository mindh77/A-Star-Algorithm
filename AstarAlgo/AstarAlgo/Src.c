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
	int dir; // ������ ���� ���� �� �߰� (0: ��, 1: �Ʒ�, 2: ��, 3: ��)
}Ty_node;

typedef struct linknode * Ty_linknodeptr;
typedef struct linknode {
	Ty_nodeptr nodeptr;
	Ty_linknodeptr next;
}Ty_linknode;

Ty_linknodeptr open=NULL;
Ty_linknodeptr closed=NULL;
Ty_linknodeptr result_node= NULL; //���� ������ ��带 �����Ѵ�.
Ty_nodeptr cur_n;	// ���� ��� n
Ty_nodeptr goal;	// goal ���

void init();	// start ���� goal ��带 �Է¹ް� �ʱ�ȭ �����ش�.
void set_cur_n();	// ���� ��� n�� �������ش�.
void cal_suc(Ty_nodeptr n);	// ��� n�� successor���� ���Ѵ�.
void cal_f_hat(Ty_nodeptr node);	// Ty_node�� f,g,h hat�� ���Ѵ�.
void cal_h_hat(Ty_nodeptr node);
int cmp_match(Ty_nodeptr node1,Ty_nodeptr node2);	//node1�� node2�� ���ؼ� 0(�ٸ���) �Ǵ� 1(����)�� ����Ѵ�.
int is_in_open(Ty_nodeptr node);					//node�� open�� �ִ����� Ȯ���Ѵ�. 0(����.) 1(����.)
int is_in_closed(Ty_nodeptr node);					//node�� closed�� �ִ����� Ȯ���Ѵ�. 0(����.) 1(����.)
void insert_into_open(Ty_nodeptr node);				//node�� open�� f_hat�� ���� ������ �ִ´�.
void insert_into_closed(Ty_nodeptr node);			//node�� closed�� �ִ´�.
void closed_to_open(Ty_nodeptr node);				//closed�� �ִ� node�� open���� �Ű� �ش�.
int compare_f_in_open(Ty_nodeptr node);				//node�� new f�� open�� old f�� ���� new f�� old���� ������ 1�� ��ȯ�Ѵ�.
int compare_f_in_closed(Ty_nodeptr node);			//node�� new f�� closed�� old f�� ���� new f�� old���� ������ 1�� ��ȯ�Ѵ�.
void update_f_g(Ty_nodeptr node);					//open�� �ִ� node�� f_hat�� g_hat���� ������Ʈ �����ش�.
void update_pred(Ty_nodeptr node, Ty_nodeptr n);	//open�� �ִ� node�� pred���� ������Ʈ �����ش�.
void copy_tile(Ty_nodeptr origin, Ty_nodeptr result, int kind); //result�� ���� origin���� �����Ѵ�. ��, kind�� ���� ���� -1�� ���� ��(0),��(1),��(2),��(3)���� �̵���Ų��.
int goal_check(Ty_nodeptr node);					//node�� goal_node�̸� 1�� ��ȯ�ϰ� �ƴҰ�� 0�� ��ȯ�Ѵ�.
void print_result();								//closed�� �ִ� ������ �����Ͽ� ��θ� ����Ѵ�.



int main() {
	// ó�� ���� start node�� ���� ��� n���� �������� ���ش�.
	init();	
	// success(n)�� ���ϰ� open�� �־��ִ� ����. 
	//��������� open�� start node�� successor���� �� �ְ�, closed�� start node�� �� �ִ�.
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
	//ó�� ���� start node�� ���� ��� n���� �������� ���ش�. goal_init node�� goal node�� ������ �ش�.
	cur_n = start_init;
	goal = goal_init;
	//start node�� hat �������� �� �Լ� ������ ��ü������ ����� �ش�.
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
	//closed->nodeptr = start_init;	//start_init�� open�� �־��ٰ� �ٷ� closed�� �־����ٰ� �����Ѵ�.
	insert_into_open(start_init);
	system("cls");
}

// ���� ��� n�� �������ش�.
// ���� ���� ��� n�� closed�� �ְ� open�� ���� ù ��° ��带 �����Ͽ� ���ο� ���� ��带 �����Ѵ�.
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


//-1�� ������ �� �ִ� 4����(��쿡 ���� 2,3������ �� �� �ִ�.) Ÿ���� ������ ���Ѵ�.
void cal_suc(Ty_nodeptr n) {
	
	int i, j;
	Ty_nodeptr left = NULL;
	Ty_nodeptr right = NULL;
	Ty_nodeptr up = NULL;
	Ty_nodeptr down = NULL;
	int in_open=0;
	int in_closed=0;

	//-1�� ��ġ�� ���� left,up,down,right ��带 ���� ������ �ش�.
	//���� ������ ���� �ʴ� ��尡 �ִٸ� NULL�� ������ ���̴�.
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			if (n->tile[i][j] == -1) {
				if (i == 0) {
					if (j == 0) {
						//left�� up�� ������ ���� �ʴ´�.
						right = (Ty_nodeptr)malloc(sizeof(Ty_node));
						down = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, right, 3);
						copy_tile(n, down, 1);
					}
					else if (j == 3) {
						//up�� right�� ������ ���� �ʴ´�.
						left = (Ty_nodeptr)malloc(sizeof(Ty_node));
						down = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, left, 2);
						copy_tile(n, down, 1);
					}
					else {
						//up�� ������ ���� �ʴ´�.
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
						//left�� down�� ������ ���� �ʴ´�.
						right = (Ty_nodeptr)malloc(sizeof(Ty_node));
						up = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, right, 3);
						copy_tile(n, up, 0);
					}
					else {
						//left�� ������ ���� �ʴ´�.
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
						//right�� down�� ������ ���� �ʴ´�.
						left = (Ty_nodeptr)malloc(sizeof(Ty_node));
						up = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, left, 2);
						copy_tile(n, up, 0);
					}
					else {
						//down�� ������ ���� �ʴ´�.
						left = (Ty_nodeptr)malloc(sizeof(Ty_node));
						right = (Ty_nodeptr)malloc(sizeof(Ty_node));
						up = (Ty_nodeptr)malloc(sizeof(Ty_node));
						copy_tile(n, right, 3);
						copy_tile(n, left, 2);
						copy_tile(n, up, 0);
					}
				}
				else if (j == 3) {
					//right�� ������ ���� �ʴ´�.
					left = (Ty_nodeptr)malloc(sizeof(Ty_node));
					down = (Ty_nodeptr)malloc(sizeof(Ty_node));
					up = (Ty_nodeptr)malloc(sizeof(Ty_node));
					copy_tile(n, left, 2);
					copy_tile(n, up, 0);
					copy_tile(n, down, 1);
				}
				else {
					//��� ������ �ȴ�.
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

	//������ success���� open�� closed�� �ִ��� �������� ���� ������ �ൿ�� ���Ѵ�.
	if (left != NULL) {
		in_open = is_in_open(left);
		in_closed = is_in_closed(left);
		//open�� closed�� �� �� ������ open�� �ְ� pred�� n���� ������ �ش�.
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
		//open�� closed�� �� �� ������ open�� �ְ� pred�� n���� ������ �ش�.
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
		//open�� closed�� �� �� ������ open�� �ְ� pred�� n���� ������ �ش�.
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
		//open�� closed�� �� �� ������ open�� �ְ� pred�� n���� ������ �ش�.
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

//origin�� ���� result�� �����Ѵ�. ��, kind�� ���� ���� -1�� ���� ��(0),��(1),��(2),��(3)���� �̵���Ų��.
//result�� hat���鵵 ����� �ش�.
void copy_tile(Ty_nodeptr origin, Ty_nodeptr result, int kind) {
	// tile���� -1�� ���� ���� ��ġ ����
	int minus_i, minus_j;
	int i, j;
	// tile���� -1�� ���� ���� ��ġ ������ ã�´�.
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
	case 0://��
		//-1 Ÿ�� ���� -1 Ÿ���� ���� ���� ������ ��� origin Ÿ���� ������ result�� �����Ѵ�.
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
	case 1://��
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
	case 2://��
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
	case 3://��
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


// start node�� ������ ������ node���� f hat, g hat, h hat�� ���ϴ� �Լ����̴�. 
// start node�� f hat�� ���ϴ� �Լ��� ������� �ʴ´�.
void cal_f_hat(Ty_nodeptr node) {
	node->f_hat = (node->g_hat) + (node->h_hat);
}
// goal node�� parameter node�� Ÿ�� ���� ���� h_hat�� ������ �ش�.
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
//node1�� node2�� ���ؼ� 0(�ٸ���) �Ǵ� 1(����)�� ����Ѵ�.
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
//node�� open�� �ִ����� Ȯ���Ѵ�. 0(����.) 1(����.)
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
//node�� closed�� �ִ����� Ȯ���Ѵ�. 0(����.) 1(����.)
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
//node�� open�� f_hat�� ���� ������ �ִ´�.
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
//node�� closed�� �ִ´�.
void insert_into_closed(Ty_nodeptr node)			
{
	Ty_linknodeptr tmp = (Ty_linknodeptr)malloc(sizeof(Ty_linknode));
	tmp->next = closed;
	tmp->nodeptr = node;
	closed = tmp;
}
//closed�� �ִ� node�� open���� �Ű� �ش�.
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
//node�� new f�� open�� old f�� ���� new f�� old���� ������ 1�� ��ȯ�Ѵ�.
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
//node�� new f�� closed�� old f�� ���� new f�� old���� ������ 1�� ��ȯ�Ѵ�.
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
//open�� �ִ� node�� f_hat�� g_hat���� ������Ʈ �����ش�.
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
//open�� �ִ� node�� pred���� ������Ʈ �����ش�.
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
//node�� goal_node�̸� 1�� ��ȯ�ϰ� �ƴҰ�� 0�� ��ȯ�Ѵ�.
int goal_check(Ty_nodeptr node) 
{
	if (cmp_match(goal, node)) {
		return 1;
	}
	return 0;
}
//closed�� �ִ� ������ �����Ͽ� ��θ� ����Ѵ�.
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
		printf("�ѤѤѤѤѤѤѤѤѤѤѤѤѤѤѤѤѤѤѤѤѤѤѤѤѤ�\n");
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
