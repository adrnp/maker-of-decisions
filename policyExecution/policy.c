//sudo apt-get install libxml2-dev
//gcc test.c -lxml2 -I/usr/include/libxml2
//Policy readPolicy(POMDP model, char *xmlFileString){
#include <libxml/xmlreader.h>

/* For readPolicy2 */
#include <libxml/parser.h>
#include <libxml/tree.h>

#include <stdio.h>
#include <math.h>
#include <string.h>		/* For strcmp */

#include "constants.h"
#include "policyExecution.h"

#define TESTING 1

static void print_element_names(xmlNode *a_node, double[][6]);


int readPolicy(char *xmlFileString, double alphaVectors[][6], double alphaActions[])
{
	xmlDoc *doc = NULL;
	xmlNode *policyElement = NULL;

	doc = xmlReadFile(xmlFileString, NULL, 0);
	if (NULL == doc) 
	{
		printf("Error opening XML file.\n");
	}

	/* Get the root element node */
	policyElement = xmlDocGetRootElement(doc);

	print_element_names(policyElement, alphaVectors);

	/* Free the document */
	xmlFreeDoc(doc);

	/* Free global variables that might have been allocated by parser */
	xmlCleanupParser();


	/* For testing purposes */
	int testing = 0;
	if (testing)
	{
		int col;
		for (col = 0; col < 6; col++)
		{
			printf("[0][%i] = %f\n", col, alphaVectors[0][col]);
			printf("[1][%i] = %f\n\n", col, alphaVectors[1][col]);
		}
	}
	return 3;
}


// this should return alpha_vectors
static void print_element_names(xmlNode *a_node, double alphaVectors[][6])
{
	xmlNode *cur_node = NULL;
	char *contentString;
	char *token;
	int rowIndex;
	int colIndex;

	for (cur_node = a_node; cur_node; cur_node = cur_node->next)
	{
		if(cur_node->type == XML_ELEMENT_NODE)
		{

			/* If it's a vector node, get the numbers */
			if (strcmp(cur_node->name, "Vector") == 0)
			{
				rowIndex = 0;
				colIndex = atoi(xmlGetProp(cur_node, "action"));

				contentString = xmlNodeGetContent(cur_node);
				token = strtok(contentString, " ");
				alphaVectors[rowIndex][colIndex] = atof(token);

				while (token != NULL)
				{
					rowIndex ++;
					token = strtok(NULL, " ");
					if (token != NULL)
					{
						alphaVectors[rowIndex][colIndex] = atof(token);
					}
				}
			}
		}
		print_element_names(cur_node->children, alphaVectors);
	}
}


double getAction(double vector[], double array[][6])
{
	double actionUtility = 0.0;
	double bestUtility = -9999.9;
	int i;
	int j;

	int bestAction = 0;

	for (j = 0; j < NUM_VECTORS; j++)
	{
		actionUtility = 0.0;
		for (i = 0; i < VECTOR_LENGTH; i++)
		{
			actionUtility += vector[i]*array[i][j];
		}
		if (TESTING)
		{
			printf("action %i, utility = %f\n", j, actionUtility);
		}
		if (actionUtility > bestUtility)
		{
			bestUtility = actionUtility;
			bestAction = j;
		}
	}
	return bestAction;
}
