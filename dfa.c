#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct token {
	struct token *priv, *next;
	char name[255];
	bool flag;
};

static bool is_high_priority(char input)
{
	if (input == '*' || input == '/' || input == '%')
		return true;

	return false;
}

static bool is_equation(char input)
{
	if (input == '=')
		return true;

	return false;
}

static bool is_operator(char input)
{
	if (input == '+' || input == '-' ||
	    input == '*' || input == '/' ||
	    input == '%')
		return true;

	return false;
}

static bool is_alphbet(char input)
{
	if ((input >= 'a' && input <= 'z') ||
	    (input >= 'A' && input <= 'Z'))
		return true;

	return false;
}

static bool is_number(char input)
{
	if (input >= '0' && input <= '9')
		return true;

	return false;
}

static void *zalloc(size_t size)
{
	void *p = malloc(size);

	memset(p, 0, size);

	return p;
}

static void drop_space(char* input)
{
	char *search, *modify;

	search = strchr(input, ' ');
	if (!search)
		return;
	modify = search;
	drop_space(++search);
	/* Include \0 when doing the copy */
	strncpy(modify, search, strlen(search) + 1);
}

static int tree_index;

/* Use Center->left->right policy */
static int traverse_tree(struct token **tree, struct token *current)
{
	struct token *p;

	if (!current)
		return tree_index;

	//if (!is_high_priority(current->name[0])) {

	if (current->flag)
		return tree_index;
	tree[tree_index++] = current;
	current->flag = true;
	p = current->priv;
	if (p && !p->flag) {
		while (!is_operator(p->name[0]) && p->priv && !p->priv->flag)
			p = p->priv;
		traverse_tree(tree, p);
	}
	
	p = current->next;
	if (p && !p->flag) {
		while (!is_operator(p->name[0]) && p->next && !p->next->flag)
			p = p->next;
		traverse_tree(tree, p);
	}

	return tree_index;
}

int main(void)
{
	struct token *token, *head, *priv;
	struct token *tree[255];
	char input[255], *test;
	int ret, size, i;

	signal(SIGINT, exit);
	printf("Press CTRL+C to exit the program\n");

again:
	printf("\rEnter a BNF expression:");
	fgets(input, 255, stdin);
	/* Get rid of the wrapping character */
	if (strlen(input) > 0 && input[strlen(input) - 1] == '\n')
		input[strlen(input) - 1] = '\0';

	drop_space(input);

	test = input;
	priv = head = token = zalloc(sizeof(*token));

	/* Create a link list for each input elements */
	while (*test != ';' && *test != '\0') {
		i = 0;
		for (;is_alphbet(*test) || is_number(*test); test++)
			token->name[i++] = *test;

		if (!i) {
			if (is_equation(*test) || is_operator(*test)) {
				token->name[i++] = *test;
			} else {
				printf("Invalid input %c\n", *test);
				goto again;
			}
			test++;
		}

		token->name[i] = '\0';

		token = token->next = zalloc(sizeof(*token));
		token->priv = priv;
		priv = token;
	}

	if (!strlen(token->name)) {
		priv->next = NULL;
		free(token);
		priv = priv->priv;
		priv->next = NULL;
		free(priv);
	}
	i = 0;
	token = head;

	/* Fine grain the accepted expression based on the grammer */
	do {
		if (*test == '\0') {
			printf("Missing ';' at the end of the expression\n");
			goto again;
		}

		if (i == 1 && !is_equation(*token->name)) {
			printf("'=' should be placed as the second token\n");
			goto again;
		}

		if ((is_equation(*token->name) ||
		    is_operator(*token->name)) && !(i & 1)) {
			printf("Each '=' and each operator should be placed "
			       "between two numbers or variable names\n");
			goto again;
		}

		if (i != 1 && is_equation(*token->name)) {
			printf("'=' should only occur once\n");
			goto again;
		}
			
		token = token->next;
		i++;
	} while (token->next);

	size = i + 1;
	token = head;
	tree_index = 0;
	ret = traverse_tree(tree, head->next);
	if (ret != size) {
		printf("unmatched size! size = %d ret = %d\n", size, ret);
		return -EINVAL;
	}

	//for (i = 0; i < size; i++)
	//	printf("%s\n", tree[i]->name);

	int j = 0, level = 0;
	printf("\n\t\t");
	for (j = 0; j < size; level++) {
		if (j == 7)
			printf("\t");
		if (j == 3 || j == 0)
			printf("\t");
		else if (j != 1)
			printf("\t\t");
		printf("%s", tree[j++]->name);
		if (j % 2)
			printf("\n\t\t");
	}

	/* Program will not stop until CTRL + C */
	goto again;

	return 0;
}
