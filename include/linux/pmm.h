#ifndef PMM_H
#define PMM_H

#define PMM_IOC_MAGIC		'@'
#define PMM_DRIVER_ON		_IOW(PMM_IOC_MAGIC, '1', char*)
#define PMM_DRIVER_OFF		_IOW(PMM_IOC_MAGIC, '0', char*)
#define PMM_DRIVER_SLEEP	_IOW(PMM_IOC_MAGIC, 'S', char*)

typedef struct pmm_driver
{
	char name[20];
	int (*on)(void *payload);
	int (*off)(void *payload);
	int (*sleep)(void *payload);
	void *payload;
	char private;
} pmm_driver;

/* Add new driver to the list */
int pmm_add_driver(struct pmm_driver *drv);

/* Del driver from the list */
int pmm_del_driver(struct pmm_driver *drv);


#endif /* PMM_H */
